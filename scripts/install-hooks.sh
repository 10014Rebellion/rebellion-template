#!/bin/sh

if [ "$1" == "--help" ] & [ "$1" == "-h" ]; then
  echo "Usage: $0 [--keep-old-hooks | -k]"
  echo "Install git hooks for the project."
  echo "Options:"
  echo "  --keep-old-hooks, -k   Do not remove existing git hooks before installing new ones. Overlapping hooks will be overwritten regardless."
  exit 0
fi

if [ "$1" != "--keep-old-hooks" ] & [ "$1" != "-k" ]; then
    echo "Removing existing git hooks..."
    rm -vrf .git/hooks/*

    if [ $? -ne 0 ]; then
        echo "Error: Failed to remove existing git hooks. Please check the .git/hooks directory."
        exit 1
    fi
else
    echo "Keeping existing git hooks..."
fi

echo ""
echo "Installing git hooks..."

cp -vrf scripts/hooks/ .git/
if [ $? -ne 0 ]; then
  echo "Error: Failed to copy git hooks. Please check the scripts/hooks directory."
  exit 1
fi

echo ""
echo "Setting execute permissions on git hooks..."

chmod +x -v .git/hooks/**
if [ $? -ne 0 ]; then
  echo "Error: Failed to set execute permissions on git hooks."
  exit 1
fi

echo ""
echo "git hooks installed successfully"
