#!/usr/bin/env bash
set -euo pipefail

EXPECTED_REPO="https://github.com/madankumar477/ros2-realsense-obstacle-detector.git"
GITHUB_NOREPLY="242546719+madankumar477@users.noreply.github.com"

if ! command -v git >/dev/null 2>&1; then
  echo "Git is not installed. Install it with: sudo apt install git" >&2
  exit 1
fi

if [[ -d .git ]]; then
  echo "A Git repository already exists in this folder. Refusing to overwrite it." >&2
  exit 1
fi

if [[ ! -f README.md || ! -d src/realsense_obstacle_detector ]]; then
  echo "Run this script from the extracted project root (README.md and src/realsense_obstacle_detector must exist)." >&2
  exit 1
fi

echo "Target repository: ${EXPECTED_REPO}"
echo "IMPORTANT: This will publish the staged project contents to a PUBLIC GitHub repository."
read -r -p "Have you created an EMPTY PUBLIC repository under madankumar477? [y/N] " answer
[[ "$answer" =~ ^[Yy]$ ]] || { echo "Create the empty public repository first, then rerun."; exit 0; }

read -r -p "Use this repository URL? [Y/n] " use_default
if [[ "$use_default" =~ ^[Nn]$ ]]; then
  read -r -p "Paste the public repository HTTPS URL: " REPO_URL
else
  REPO_URL="$EXPECTED_REPO"
fi

if [[ -z "${REPO_URL:-}" || "$REPO_URL" != https://github.com/*/*.git ]]; then
  echo "Expected an HTTPS GitHub URL ending in .git" >&2
  exit 1
fi

# Use GitHub's noreply address so a personal/work email is not exposed in commit metadata.
git config user.name "E MADAN KUMAR"
git config user.email "$GITHUB_NOREPLY"

git init -b main
git add -A

git diff --cached --check

echo
echo "Review the exact files staged for PUBLIC upload:"
git status --short
echo
git diff --cached --stat
echo
read -r -p "Create the initial commit and push these files to the PUBLIC repository? [y/N] " confirm
[[ "$confirm" =~ ^[Yy]$ ]] || { echo "Cancelled. Nothing was pushed."; exit 0; }

git commit -m "Initial commit: ROS 2 RealSense obstacle detector"
git remote add origin "$REPO_URL"
git push -u origin main
echo
echo "Upload complete. Open the repository and verify visibility and rendered assets."
