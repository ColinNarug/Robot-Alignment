# GitHub + Linux

Repository:

```bash

https://github.com/ColinNarug/Robot-Alignment

```

This guide sets up GitHub access from an Ubuntu terminal using **HTTPS**, a **GitHub username**, and a **token-backed login**. It covers installing Git and GitHub CLI, authenticating, cloning the repository, tracking changes, committing, pushing, and using branches.

---

## 0. Minimal daily workflow after setup is complete

After everything is already installed, authenticated, and cloned, the normal repeated workflow is much shorter.

```bash

cd ~/Robot-Alignment

git status

git pull
  

# edit files

  
git status

git diff

git add .

git diff --staged

git commit -m "Describe the change"

git push

```

  

---
## 1. Install Git and basic terminal tools

This installs the basic software needed before working with GitHub from the terminal.

- `git` tracks file changes and communicates with GitHub repositories.

- `wget` downloads files from the internet.

- `gpg` verifies signing keys.

- `ca-certificates` helps Ubuntu verify HTTPS connections.

Git is required for cloning, tracking, committing, pulling, and pushing. The other tools are needed to install the official GitHub CLI package source securely.

Run this from **any directory**.

```bash

sudo apt update

sudo apt install -y git wget gpg ca-certificates

```

Check that Git installed:

```bash

git --version

```

Expected output style:
git version 2.53.0
The exact version may be different.

---
## 2. Install the official GitHub CLI package

Run this from **any directory**.

```bash

(type -p wget >/dev/null || (sudo apt update && sudo apt install wget -y)) \
&& sudo mkdir -p -m 755 /etc/apt/keyrings \
&& out=$(mktemp) \
&& wget -nv -O"$out" https://cli.github.com/packages/githubcli-archive-keyring.gpg \
&& cat "$out" | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null \
&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg \
&& sudo mkdir -p -m 755 /etc/apt/sources.list.d \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt install gh -y

```


Verify installation:

```bash

gh --version

```

Expected output style:
gh version 2.92.0
The exact version may be different.

To check where Ubuntu is getting `gh` from:

```bash

apt-cache policy gh

```

A setup using the official GitHub CLI repository should show a source similar to:

```text

https://cli.github.com/packages stable/main

```

---
## 3. Configure Git identity
### What this is

This tells Git what name and email address to attach to commits.

A Git commit records information like:

Author name

Author email

Commit message

File changes

Timestamp

### Why this is needed

Without a configured name and email, Git may refuse to create commits or may attach the wrong identity to commits.

This identity is not necessarily the same as the GitHub username. It is the author identity written into commits.

  
Run this from **any directory**.

Replace the name and email with the correct identity:

```bash

git config --global user.name "Your Name"

git config --global user.email "your_email@example.com"

```

  

Set the default branch name for new local repositories:

```bash

git config --global init.defaultBranch main

```

  
Set the default pull behavior:

```bash

git config --global pull.rebase false

```

  
Check the global Git settings:

```bash

git config --global --list

```

  
  

---

  

## 4. Log into GitHub from the terminal using HTTPS

### What this is

This connects the Ubuntu terminal to a GitHub account using GitHub CLI and configures Git operations to use **HTTPS**.

### Why this is needed

Cloning public repositories can work without authentication, but pushing changes to GitHub requires authentication.

Run this from **any directory**.

```bash

gh auth login

```

  
Typical prompt choices:

GitHub.com

HTTPS

Login with a web browser

Authenticate Git with your GitHub credentials: Y

After login, explicitly configure Git to use GitHub CLI as the credential helper:

```bash

gh auth setup-git

```


Check authentication status:

```bash

gh auth status

```


Confirm that GitHub CLI can read the repository information:

```bash

gh repo view ColinNarug/Robot-Alignment

```


---

  
## 5. Clone the repository directly with HTTPS

Use this path if the user has write access to the original repository.


### What this is

Cloning downloads the GitHub repository to the local Ubuntu machine.

### Why this is needed

GitHub stores the remote copy of the project. To edit files and create commits locally, there must be a local copy on the computer.

Run this from the folder where the repository folder should be created.

For a normal setup, use the home directory:

```bash

cd ~

```

  
Clone the repository using HTTPS:

```bash

git clone https://github.com/ColinNarug/Robot-Alignment.git

```


Enter the repository:

```bash

cd ~/Robot-Alignment

```
  

Check the remote connection:

```bash

git remote -v

```


Expected style of output:

origin https://github.com/ColinNarug/Robot-Alignment.git (fetch)

origin https://github.com/ColinNarug/Robot-Alignment.git (push)


---

  
## 6. Pull latest changes before editing

### What this is

`git pull` downloads the newest commits from GitHub and integrates them into the local repository.
### Why this is needed

If the remote repository changed since the local copy was cloned, pulling prevents working from an outdated version.

Must be run inside the repository:

```bash

cd ~/Robot-Alignment

```

Check local state first:

```bash

git status

```

  Useful related checks:

```bash

git branch

git remote -v

```

Where `origin` is the original repository:

```bash

git pull

```

---

## 7. Edit Files

### What this is

This is where the user makes actual changes to the project files.
### Why this is needed

Git only tracks changes that actually happen inside the repository folder.

Files should be edited inside:

```bash

~/Robot-Alignment

```

Example terminal navigation: 
cd ~/Robot-Alignment
ls

Open the repository folder in a file manager:

```bash

xdg-open .

```

Open the repository in VS Code, if VS Code is installed:

```bash

code .

```

After editing, return to the terminal and check status:

```bash

git status

```

  

---

## 8. View file changes

### What this is

`git diff` shows exactly what changed inside files.
### Why this is needed

This prevents committing accidental edits. It lets the user inspect changes before staging or committing them.

Must be run inside the repository:

```bash

cd ~/Robot-Alignment

```

Show unstaged changes:

```bash

git diff

```

Show staged changes:

```bash

git diff --staged

```

Show recent commit history:

```bash

git log --oneline --graph --decorate --all -n 10

```

---
## 9. Stage local changes
### What this is

Staging means selecting which changed files should be included in the next commit.

The staging command is: git add

### Why this is needed

Git does not automatically commit every changed file. Staging gives the user control over exactly what goes into the next commit.

Must be run inside the repository:

```bash

cd ~/Robot-Alignment

```

  
Check changes:

```bash

git status

```

  
Stage one file:

```bash

git add path/to/file

```

Example: git add README.md

Stage all changed files:

```bash

git add .

```

Check what is staged:

```bash

git status

```

Review staged changes:

```bash

git diff --staged

```

---

## 10. Commit local changes

### What this is

A commit is a saved checkpoint in the repository history.

It stores the staged changes with a message explaining what changed.
### Why this is needed

Pushing sends commits to GitHub. If the user has only edited files but has not committed them, there is nothing permanent to push.
Must be run inside the repository:

```bash

cd ~/Robot-Alignment

```

Commit staged changes:

```bash

git commit -m "Describe the change clearly"

```

Example: git commit -m "Add GitHub terminal setup instructions"  

Check recent commits:

```bash

git log --oneline -n 5

```

---

## 11. Push changes to GitHub
### What this is

`git push` uploads local commits to GitHub.
### Why this is needed

A commit only exists on the local machine until it is pushed. Pushing updates the GitHub repository or fork.

Must be run inside the repository:

```bash

cd ~/Robot-Alignment

```
### Push directly to the original repository

Use this if the user has write access to `ColinNarug/Robot-Alignment`.

For an existing branch that already tracks a remote branch:

```bash

git push

```

---

  

## 12. Good-practice commands

These are not all required every time, but they are useful for avoiding mistakes.
### Check what changed

#### What it is

Shows current file status and unstaged changes.
#### Why it is used

Use this before staging or committing.

Inside the repo:

```bash

cd ~/Robot-Alignment

```
Commands:
```bash

git status

git diff

```
---

### Unstage a file but keep the edits
#### What it is

Removes a file from the staged area.
#### Why it is used

Useful if a file was accidentally added with `git add`.

Inside the repo:

```bash

cd ~/Robot-Alignment

```
Command:
```bash

git restore --staged path/to/file

```

Example:  git restore --staged README.md  

---

### Discard local edits to one file
#### What it is
Restores a file back to the last committed version.
#### Why it is used

Useful when edits should be thrown away.

Be careful: this deletes uncommitted changes in that file.
Inside the repo:

```bash

cd ~/Robot-Alignment

```
Command:

```bash

git restore path/to/file

```

Example: git restore README.md

---
### Save unfinished work temporarily

#### What it is

`git stash` temporarily shelves uncommitted changes.
#### Why it is used

Useful when the user needs to pull updates or switch branches but does not want to commit unfinished work.
Inside the repo:

```bash

cd ~/Robot-Alignment

```
Commands:
```bash

git stash push -m "temporary work"

git stash list

git stash pop

```

---
### See commit history
#### What it is

Shows previous commits.
#### Why it is used

Useful for checking what changed recently and finding commit IDs.
Inside the repo:

```bash

cd ~/Robot-Alignment

```
Command:
```bash

git log --oneline --graph --decorate --all

```

---

### Check the GitHub remote
#### What it is

Shows which GitHub repository the local repository is connected to.

#### Why it is used

Prevents accidentally pushing to the wrong repository or using SSH when HTTPS is intended.
Inside the repo:
```bash

cd ~/Robot-Alignment

```
Command:
```bash

git remote -v

```


### Remove GitHub User from Local Machine
#### What it is

This removes the local GitHub login and Git author identity from the machine.

#### Why it is used

Useful when a different person will use the computer, or when the GitHub account should no longer be connected to local terminal GitHub access.  
This does not delete local repository folders or project files.

```bash  
gh auth status
```

Log out of GitHub CLI Locally:
```bash  
gh auth logout --hostname github.com
```

Remove the gloabal Git commit identity:
```bash  
git config --global --unset-all user.name
git config --global --unset-all user.email
```

Remove the GitHub CLI credential-helper connection from global Git config:
```bash  
git config --global --unset-all credential.https://github.com.helper
git config --global --unset-all credential.https://gist.github.com.helper
```

---
## 18. Source notes

  

- GitHub CLI Linux install instructions: https://github.com/cli/cli/blob/trunk/docs/install_linux.md

- GitHub CLI `gh auth login` manual: https://cli.github.com/manual/gh_auth_login

- GitHub CLI `gh auth setup-git` manual: https://cli.github.com/manual/gh_auth_setup-git

- GitHub Docs: Caching GitHub credentials in Git: https://docs.github.com/en/get-started/git-basics/caching-your-github-credentials-in-git

- GitHub Docs: Managing personal access tokens: https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens

- GitHub Docs: Troubleshooting cloning errors: https://docs.github.com/en/repositories/creating-and-managing-repositories/troubleshooting-cloning-errors
