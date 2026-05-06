# GitHub + Linux

Repository:
```bash
https://github.com/ColinNarug/Robot-Alignment
```
This guide sets up GitHub access from an Ubuntu terminal using **HTTPS**, a **GitHub username**, and a **token-backed login**. It covers installing Git and GitHub CLI, authenticating, cloning the repository, tracking changes, committing, pushing, and using branches.

---
## 0. Directory rules
Some commands can be run from **any directory**. Other commands must be run from **inside the repository folder**.
### Commands that can be run from anywhere
```bash
git --version
gh --version
git config --global ...
gh auth login
gh auth status
gh auth setup-git
```
These commands either check installed software, configure the whole Linux user account, or authenticate with GitHub.

### Commands that must be run inside the repository
```bash
git status
git add
git commit
git push
git pull
git diff
git log
git branch
git checkout
```

These commands operate on a specific Git repository. For this project, the repository folder will normally be:
```bash
~/Robot-Alignment
```

Before doing normal Git work, use:
```bash
cd ~/Robot-Alignment
```

### Important note about `git config --global`
`git config --global` applies to the entire Linux user account, not just the current directory.
That means this can be run from anywhere:

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
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
     | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
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
This guide uses HTTPS because it matches the familiar GitHub clone URL format:
https://github.com/OWNER/REPOSITORY.git

GitHub CLI stores and supplies the authentication token so the user does not have to manually paste a token every time.


Run this from **any directory**.
```bash
gh auth login --hostname github.com --git-protocol https --web
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

### 4.1 Manual token fallback method

### What this is
This is the backup method if GitHub CLI authentication is not used.
The user clones or pushes using an HTTPS URL. When Git asks for credentials:
```text
Username: GitHub username
Password: personal access token, not the GitHub account password
```

### Why this is needed

GitHub does not accept normal account passwords for HTTPS Git operations. The token acts as the password for Git operations.

The clone command should be run from the folder where the repository should be created. Normal location:
```bash
cd ~
```

Clone with HTTPS:
```bash
git clone https://github.com/ColinNarug/Robot-Alignment.git
```

If prompted:
```text
Username: YOUR-GITHUB-USERNAME
Password: YOUR-PERSONAL-ACCESS-TOKEN
```

### Token safety rules

Do **not** paste a token into a command like this:
```bash
git clone https://TOKEN@github.com/ColinNarug/Robot-Alignment.git
```
Do **not** save a token in a script, notes file, README, or repository file.
Prefer GitHub CLI for normal users because it handles token storage more cleanly.

---

## 6. Clone the repository directly with HTTPS
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
origin  https://github.com/ColinNarug/Robot-Alignment.git (fetch)
origin  https://github.com/ColinNarug/Robot-Alignment.git (push)




---

## 7. Check repository status

### What this is
`git status` shows the current state of the repository.

It reports files as:
Untracked
Modified
Staged
Committed
Ready to push


### Why this is needed
This is the main safety-check command. It should be used before adding, committing, pulling, or pushing so the user knows exactly what Git sees.


Must be run inside the repository:
```bash
cd ~/Robot-Alignment
```

Run:
```bash
git status
```

Useful related checks:
```bash
git branch
git remote -v
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

## 9. Pull latest changes before editing

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

Where `origin` is the original repository:
```bash
git pull
```


Then push the updated `main` branch to the fork:
```bash
git push origin main
```

---

## 10. Create a safer work branch

### What this is
A branch is a separate line of development.
Instead of changing `main` directly, a user can create a branch for one specific fix, feature, or documentation change.

### Why this is needed
Branches make the workflow safer. They keep `main` clean and make it easier to review, test, discard, or submit changes.


Must be run inside the repository:
```bash
cd ~/Robot-Alignment
```

Make sure `main` is selected:
```bash
git checkout main
```

Update `main`.
```bash
git pull
```

Create a new branch:
```bash
git checkout -b feature/descriptive-name
```

Example:
```bash
git checkout -b docs/github-terminal-setup
```

Check the current branch:
```bash
git branch
```

The active branch will have an asterisk:
```text
* docs/github-terminal-setup
  main
```

---

## 11. Edit files

### What this is
This is where the user makes actual changes to the project files.

### Why this is needed
Git only tracks changes that actually happen inside the repository folder.


Files should be edited inside:
```bash
~/Robot-Alignment
```

Example terminal navigation:
```bash
cd ~/Robot-Alignment
ls
```

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

## 12. Stage local changes

### What this is
Staging means selecting which changed files should be included in the next commit.

The staging command is:
```bash
git add
```

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
Example:
```bash
git add README.md
```

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

## 13. Commit local changes

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

Example:
```bash
git commit -m "Add GitHub terminal setup instructions"
```

Check recent commits:
```bash
git log --oneline -n 5
```

---

## 14. Push changes to GitHub

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

For a new branch:
```bash
git push -u origin docs/github-terminal-setup
```
Replace the branch name as needed.

If Git asks for credentials:
```text
Username: YOUR-GITHUB-USERNAME
Password: YOUR-PERSONAL-ACCESS-TOKEN
```

If GitHub CLI authentication was set up correctly, it should usually handle this without manual token entry.


---

## 15. Create a pull request

### What this is
A pull request asks the original repository to review and accept changes from a branch.

### Why this is needed
For users who do not have direct write access, a pull request is the standard way to propose changes. For users who do have write access, pull requests are still useful because they allow review before merging into `main`.


Must be run inside the repository:
```bash
cd ~/Robot-Alignment
```

Create a pull request from the current branch:
```bash
gh pr create
```

Or create it with title and body in one command:
```bash
gh pr create --base main --title "Add GitHub terminal setup instructions" --body "Adds end-to-end Ubuntu terminal instructions for GitHub setup and repository workflow."
```

Check pull request status:
```bash
gh pr status
```

Open the pull request in a browser:
```bash
gh pr view --web
```


---

## 16. Good-practice commands
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
Example:
```bash
git restore --staged README.md
```

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
Example:
```bash
git restore README.md
```

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


---

## 17. Minimal daily workflow after setup is complete
After everything is already installed, authenticated, and cloned, the normal repeated workflow is much shorter.
```bash
cd ~/Robot-Alignment

git checkout main
git pull

git checkout -b docs/my-change

# edit files

git status
git diff
git add .
git diff --staged
git commit -m "Describe the change"
git push -u origin docs/my-change
```


---

## 18. Source notes

- GitHub CLI Linux install instructions: https://github.com/cli/cli/blob/trunk/docs/install_linux.md
- GitHub CLI `gh auth login` manual: https://cli.github.com/manual/gh_auth_login
- GitHub CLI `gh auth setup-git` manual: https://cli.github.com/manual/gh_auth_setup-git
- GitHub Docs: Caching GitHub credentials in Git: https://docs.github.com/en/get-started/git-basics/caching-your-github-credentials-in-git
- GitHub Docs: Managing personal access tokens: https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens
- GitHub Docs: Troubleshooting cloning errors: https://docs.github.com/en/repositories/creating-and-managing-repositories/troubleshooting-cloning-errors
