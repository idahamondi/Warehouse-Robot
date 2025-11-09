# Git Commands for Uploading to GitHub

## Step 1: Create Initial Commit (if not done)

```bash
git commit -m "Initial commit: Warehouse Robot with autonomous navigation, obstacle avoidance, and robotic arm control"
```

## Step 2: Create a GitHub Repository

1. Go to https://github.com and sign in
2. Click the "+" icon in the top right corner
3. Select "New repository"
4. Name your repository (e.g., "Warehouse-Robot" or "Arduino-Warehouse-Robot")
5. Choose Public or Private
6. **DO NOT** initialize with README, .gitignore, or license (we already have these)
7. Click "Create repository"

## Step 3: Connect Local Repository to GitHub

After creating the repository on GitHub, you'll see instructions. Use these commands:

```bash
# Add the remote repository (replace YOUR_USERNAME and REPO_NAME with your actual values)
git remote add origin https://github.com/YOUR_USERNAME/REPO_NAME.git

# Verify the remote was added
git remote -v
```

## Step 4: Push to GitHub

```bash
# Push to GitHub (first time)
git branch -M main
git push -u origin main
```

## Alternative: If you already have a main branch

```bash
# Check current branch name
git branch

# If branch is 'master', rename it to 'main' (GitHub's default)
git branch -M main

# Push to GitHub
git push -u origin main
```

## Future Updates

After making changes to your code:

```bash
# Stage all changes
git add .

# Commit changes
git commit -m "Description of your changes"

# Push to GitHub
git push
```

## Troubleshooting

### If you get authentication errors:
- GitHub no longer accepts passwords for HTTPS. You need to:
  1. Use a Personal Access Token (PAT) instead of password
  2. Or use SSH instead of HTTPS

### To use SSH instead:
```bash
# Change remote URL to SSH (replace YOUR_USERNAME and REPO_NAME)
git remote set-url origin git@github.com:YOUR_USERNAME/REPO_NAME.git

# Then push
git push -u origin main
```

### To use Personal Access Token:
1. Go to GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Generate a new token with `repo` permissions
3. Use the token as your password when pushing

---

**Quick Reference - All commands in sequence:**

```bash
# 1. Commit (if not already committed)
git commit -m "Initial commit: Warehouse Robot with autonomous navigation, obstacle avoidance, and robotic arm control"

# 2. Add remote (replace with your repo URL)
git remote add origin https://github.com/YOUR_USERNAME/REPO_NAME.git

# 3. Rename branch to main (if needed)
git branch -M main

# 4. Push to GitHub
git push -u origin main
```

