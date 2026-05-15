# How to Prepare an External Ubuntu Drive for Clonezilla Cloning

## Disclaimer: It is YOUR responsibility to ensure you are wiping the contents of the intended storage device! Specifying the wrong drive during this process WILL delete everything on it and cannot be recovered! If possible, remove all storage devices not part of the cloning process to reduce this risk. Create backups of anything that cannot be removed PRIOR to this process and keep it unplugged and separate. Note: The risk is Not from a faulty process but from erasing the contents of the wrong storage device.

This guide documents the preparation steps needed before using Clonezilla to clone an existing external Ubuntu installation onto empty target drives.

It covers:

1. Preparing the external Ubuntu source drive.
2. Shrinking the source installation so it can fit on smaller target drives.
3. Preparing a USB image repository drive.
4. Creating a Clonezilla boot USB.
5. Reaching the point where Clonezilla can be used to clone an empty drive.

The actual Clonezilla save/restore operation can be documented separately with the recorded video. Or, follow the Clonezilla online documentation: https://clonezilla.org//clonezilla-live-doc.php

---

## 0. Required hardware

| Device | Purpose |
|---|---|
| External Ubuntu drive | The source Ubuntu OS that will be cloned |
| USB repository drive | Temporary storage for Clonezilla image files |
| Clonezilla boot USB | Boots the Clonezilla Live environment |
| Empty target drive | The drive that will receive the cloned Ubuntu OS |

Example device roles from the completed workflow:

| Device | Role |
|---|---|
| Inland 1TB external SSD | Original external Ubuntu master |
| PNY 256GB USB | Temporary Clonezilla image repository |
| Verbatim 128GB USB | Clonezilla boot USB, then later reused as a target |
| Other Verbatim 128GB USBs | Empty target drives for the cloned Ubuntu OS |

Only three USB devices need to be connected at once:

Clonezilla boot USB + image repository USB + source/target drive

---

## 1. Identify the external Ubuntu source drive

Boot into the external Ubuntu installation that will become the master/source.

Open Terminal and run:

```bash
echo "=== DISK LAYOUT ==="
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS

echo
echo "=== ROOT FILESYSTEM USAGE ==="
df -h /

echo
echo "=== HOME FOLDER USAGE ==="
sudo du -sh /home/* 2>/dev/null
```

Confirm the external Ubuntu drive has:

EFI partition       usually FAT32/vfat, mounted at /boot/efi
Ubuntu root         ext4, mounted at /

Example source layout:


sda      953.9G Inland
├─sda1       1G vfat   /boot/efi
└─sda2      95G ext4   /


The key checks are:

1. Which disk is the external Ubuntu source.
2. Which partition is the EFI partition.
3. Which partition is the Ubuntu root partition.
4. How much space is actually used on `/`.

---

## 2. Clean the source Ubuntu system before cloning

Still booted into the source Ubuntu system, run:

```bash
echo "=== CLEANING APT PACKAGES ==="
sudo apt autoremove -y
sudo apt clean

echo
echo "=== CLEANING OLD JOURNAL LOGS ==="
sudo journalctl --vacuum-time=7d

echo
echo "=== CLEANING USER CACHE ==="
rm -rf ~/.cache/*

echo
echo "=== CHECK ROOT USAGE AFTER CLEANUP ==="
df -h /
```

If the clone will be distributed to other people, review whether the source system should keep or remove:

- browser profiles
- saved passwords
- SSH keys
- GitHub credentials
- personal documents
- Downloads folder contents
- machine-specific project files

A literal clone keeps all of this. A distributable image usually should not.

---

## 3. Decide the target partition size

The source Ubuntu root partition must be small enough to fit on the target drives.

For 128GB target USB sticks, a safe root partition size is about: 95 GiB

That equals: 97280 MiB

A nominal 128GB USB stick typically appears in Linux as roughly: 115–119 GiB

So this layout leaves enough margin:
EFI partition:      about 1 GiB
Ubuntu root:        about 95 GiB
Remaining space:    unallocated


---

## 4. Shrink the external Ubuntu source drive

Do not shrink the source drive while booted from it.

Use another Ubuntu system, Ubuntu Live USB, or GParted Live USB.

### 4.1 Boot into a separate system

Boot into a different Ubuntu installation or a live USB.

Plug in only the external Ubuntu source drive.

Run:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Identify the external Ubuntu drive by model and size.
Example:
sdc     953.9G Inland
├─sdc1      1G vfat
└─sdc2  952.8G ext4


### 4.2 Unmount the source root partition

If the root partition auto-mounted, unmount it:

```bash
sudo umount /dev/sdX2
```

Example:

```bash
sudo umount /dev/sdc2
```

Verify that it is unmounted:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

The source root partition should have no mountpoint.

---

## 5. Install and open GParted

Check whether GParted is installed:

```bash
which gparted
```

If nothing prints, install it:

```bash
sudo apt update
sudo apt install -y gparted
```

Open GParted:

```bash
sudo gparted
```

In GParted:

1. Select the external Ubuntu source drive from the top-right device selector.
2. Do not select the internal system drive.
3. Right-click the ext4 Ubuntu root partition.
4. Choose **Resize/Move**.
5. Set the new size to:

```text
97280 MiB
```

6. Leave the EFI partition unchanged.
7. Click **Resize/Move**.
8. Click the green checkmark to **Apply All Operations**.
9. Wait for the operation to complete.

Expected layout after shrinking:
External Ubuntu source drive
├─ EFI partition       about 1 GiB
├─ Ubuntu root         about 95 GiB
└─ unallocated         remaining space


---

## 6. Check the resized filesystem

After GParted finishes, close GParted and run:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Confirm the root partition is now about 95G.

Then run a filesystem check while the root partition is still unmounted:

```bash
sudo e2fsck -f /dev/sdX2
```

Example:
sudo e2fsck -f /dev/sdc2

If there are no error prompts, the resized filesystem is healthy.

---

## 7. Boot-test the shrunken external Ubuntu source

Before creating any Clonezilla image, confirm the shrunken source still boots.

1. Shut down.
2. Plug in only the external Ubuntu source drive.
3. Boot from that external drive using the computer's one-time boot menu.
   - MSI boot menu key: `F11`
   - Many Dell systems use `F12`
4. Once Ubuntu boots, run:

```bash
echo "=== DISK LAYOUT AFTER BOOTING FROM SHRUNKEN SOURCE ==="
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS

echo
echo "=== ROOT MOUNT ==="
findmnt /

echo
echo "=== EFI MOUNT ==="
findmnt /boot/efi

echo
echo "=== ROOT USAGE ==="
df -h /
```

Correct result:
/ is mounted from the external Ubuntu drive's ext4 partition
/boot/efi is mounted from the external Ubuntu drive's vfat partition
root size is approximately the new resized size

Do not proceed until the shrunken external Ubuntu source successfully boots.

---

## 8. Prepare the USB image repository drive

The image repository drive is where Clonezilla will save the image files.

### 8.1 Identify the repository USB

Plug in only the repository USB.

Run:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Identify the drive by size and model.

Example:
sdb     232.2G USB
└─sdb1  232.2G exfat PNY_USB

### 8.2 Format the repository USB as ext4

This erases the repository USB.

Replace `/dev/sdX` with the actual repository disk, not the partition.

```bash
sudo umount /dev/sdX* 2>/dev/null

sudo wipefs -a /dev/sdX

sudo parted -s /dev/sdX mklabel gpt
sudo parted -s /dev/sdX mkpart CZ_REPO ext4 1MiB 100%

sudo mkfs.ext4 -F -L CZ_REPO /dev/sdX1

sync
```

Verify:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Expected:
sdX     232.2G USB
└─sdX1  232.2G ext4  CZ_REPO

The label `CZ_REPO` makes the repository easy to identify inside Clonezilla.

---

## 9. Create a Clonezilla boot USB

The Clonezilla boot USB is the tool used to boot into Clonezilla Live.

It is separate from:

- the external Ubuntu source drive
- the repository USB
- the target drive

If there is no extra USB stick, one target USB can temporarily be used as the Clonezilla boot USB and overwritten last.

### 9.1 Download the Clonezilla ISO

From Ubuntu, download the Clonezilla Live ISO.

Example:
cd ~/Downloads
wget -O clonezilla-live-3.3.1-35-amd64.iso \
https://free.nchc.org.tw/clonezilla-live/stable/clonezilla-live-3.3.1-35-amd64.iso
ls -lh clonezilla-live-3.3.1-35-amd64.iso

### 9.2 Identify the USB that will become the Clonezilla boot USB

Plug in the USB stick that will become Clonezilla Live.

Run:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Identify the USB carefully.

Example:
sdc     115.5G STORE
└─sdc1  115.5G vfat STORE N GO

### 9.3 Write the Clonezilla ISO to the USB

This erases the selected USB.

Replace `/dev/sdY` with the actual Clonezilla boot USB disk, not a partition.

```bash
sudo umount /dev/sdY* 2>/dev/null

sudo dd if=~/Downloads/clonezilla-live-3.3.1-35-amd64.iso of=/dev/sdY bs=4M status=progress oflag=sync

sync
```

Verify:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Expected:
sdY     115.5G STORE  iso9660  3.3.1-35-amd64
└─sdY1    463M        iso9660  3.3.1-35-amd64

That means the USB now boots Clonezilla.

---

## 10. Final pre-Clonezilla checklist

At this point, the system is ready for the Clonezilla operation.
You should have:
```text
1. External Ubuntu source drive
   - boot-tested after shrinking
   - EFI partition intact
   - root partition resized small enough for the target drives

2. Repository USB
   - formatted ext4
   - labeled CZ_REPO
   - large enough to store the image

3. Clonezilla boot USB
   - written from the Clonezilla ISO
   - boots to Clonezilla Live

4. Empty target drive
   - equal to or larger than the cloned partition layout
```

---

## 11. What the Clonezilla video should cover

[https://clonezilla.org//clonezilla-live-doc.php](https://clonezilla.org//fine-print-live-doc.php?path=clonezilla-live/doc/03_One_image_to_multiple_disks)
The Clonezilla operation documentation/video should show:
### Image creation
```text
Boot Clonezilla
→ Start Clonezilla
→ device-image
→ local_dev
→ select CZ_REPO as /home/partimag
→ savedisk
→ select external Ubuntu source disk
→ save image to repository
→ verify image
```

### First restore to target drive
```text
Boot Clonezilla
→ Start Clonezilla
→ device-image
→ local_dev
→ select CZ_REPO as /home/partimag
→ restoredisk
→ select saved image
→ select empty target drive
→ restore image
```

### If the target is smaller than the original physical source disk

If the source drive was originally larger than the target drive, Clonezilla may report:
Destination disk is too small

This can happen even after shrinking the source partitions, because the saved disk image may still record the original physical source disk size.

In that case, use:
Expert mode
→ restoredisk
→ enable -icds

Use the partition table from the image.
Do not use proportional partitioning if it would shrink the already-resized root partition unexpectedly.

### Recommended production workflow after first successful target

After one target drive has been restored and boot-tested successfully, it can be used as a gold source:
First working target clone
→ savedisk
→ gold target-sized image stored on repository USB

Then restore that target-sized gold image to the remaining empty target drives:
gold image on repository USB
→ restoredisk
→ empty target drive

---

## 12. Boot-test every completed clone

After restoring an image to a target drive, boot from that target drive.

Run:

```bash
echo "=== DISK LAYOUT FROM CLONE ==="
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS

echo
echo "=== ROOT MOUNT ==="
findmnt /

echo
echo "=== EFI MOUNT ==="
findmnt /boot/efi

echo
echo "=== ROOT USAGE ==="
df -h /
```

Correct result:
/ is mounted from the target drive's ext4 partition
/boot/efi is mounted from the target drive's vfat partition
root size matches the expected cloned layout

Do not assume a clone works until it has booted successfully.

---

## 13. Optional: wipe the repository USB after cloning

After all target clones are complete and boot-tested, the repository USB can be wiped and reused.

Example: reformat it as exFAT for general storage.

First identify it:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Then replace `/dev/sdX` with the actual repository USB disk:

```bash
sudo umount /dev/sdX* 2>/dev/null

sudo wipefs -a /dev/sdX

sudo parted -s /dev/sdX mklabel gpt
sudo parted -s /dev/sdX mkpart PNY_STORAGE 1MiB 100%

sudo partprobe /dev/sdX
sleep 2

sudo mkfs.exfat -n PNY_STORAGE /dev/sdX1

sync
```

Verify:

```bash
lsblk -o NAME,SIZE,MODEL,SERIAL,FSTYPE,LABEL,MOUNTPOINTS
```

Expected:
sdX     232.2G USB
└─sdX1  232.2G exfat  PNY_STORAGE

---

## 14. Critical warnings

### Do not trust device letters blindly

Device names can change:
/dev/sda
/dev/sdb
/dev/sdc

Always identify by:

- size
- model
- serial
- partition layout
- filesystem label
- mountpoint

### Do not overwrite the wrong drive

Never select the internal drive as a Clonezilla destination.

Internal drives usually appear as: nvme0n1

or something similar.
### Confirm the destination before every restore

Clonezilla restore operations are destructive.

Before confirming, verify the target is the intended empty drive.

### Keep the original source until all clones are tested

Do not modify or wipe the original external Ubuntu source drive until all cloned drives have been boot-tested.

---

## 15. Summary of the preparation workflow

```text
1. Boot the external Ubuntu source.
2. Check disk layout and root usage.
3. Clean the source system.
4. Decide target root partition size.
5. Boot from another system.
6. Shrink the source root partition with GParted.
7. Check the resized filesystem with e2fsck.
8. Boot-test the shrunken source.
9. Format the repository USB as ext4 labeled CZ_REPO.
10. Download the Clonezilla ISO.
11. Write the Clonezilla ISO to a boot USB.
12. Boot Clonezilla and perform the recorded image/restore workflow.
13. Boot-test every completed clone.
```