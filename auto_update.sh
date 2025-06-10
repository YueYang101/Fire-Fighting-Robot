#!/bin/bash

# Auto Update Script for Fire-Fighting-Robot
# Place this script in the Fire-Fighting-Robot repository

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "🔄 Checking for updates..."

# Navigate to the repository
cd "$SCRIPT_DIR"

# Fetch latest changes
git fetch

# Check if we're behind the remote
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse @{u})

if [ $LOCAL != $REMOTE ]; then
    echo "📥 Updates available, pulling..."
    git pull
    echo "✅ Updated to latest version"
else
    echo "✅ Already up to date"
fi
