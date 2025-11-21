import os
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms


class HolesDataset(Dataset):
    def __init__(self, root_path: str, in_channels: int = 3, resize=(256, 256)):
        """
        root_path: dataset root folder (must contain 'roi/' and 'masks/')
        in_channels: 3 = RGB, 1 = grayscale
        resize: tuple (H, W) for resizing images and masks
        """
        self.images = sorted([os.path.join(root_path, "roi", i) for i in os.listdir(os.path.join(root_path, "roi"))])
        self.masks  = sorted([os.path.join(root_path, "masks", i) for i in os.listdir(os.path.join(root_path, "masks"))])
        self.in_channels = in_channels

        self.transform = transforms.Compose([
            transforms.Resize(resize), # resize images and masks to the given size (256x256)
            transforms.ToTensor()      # outputs in [0,1]
        ])

    def __getitem__(self, index):
        if self.in_channels == 3:
            img = Image.open(self.images[index]).convert("RGB") # color
        else:  # in_channels == 1
            img = Image.open(self.images[index]).convert("L")   # grayscale

        mask = Image.open(self.masks[index]).convert("L")

        return self.transform(img), self.transform(mask)

    def __len__(self):
        return len(self.images)
