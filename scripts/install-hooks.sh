#!/bin/sh

echo "Setting execute permissions on git hooks..."

chmod +x -v scripts/hooks/**
if [ $? -ne 0 ]; then
  echo "Error: Failed to set execute permissions on git hooks."
  exit 1
fi

echo ""
echo "Installing git hooks..."

cp -vrfa scripts/hooks/ .git/
if [ $? -ne 0 ]; then
  echo "Error: Failed to copy git hooks. Please check the scripts/hooks directory."
  exit 1
fi

echo ""
echo "git hooks installed successfully"

echo "installing hooks for RebeLib..."

chmod +x -v scripts/rebelib/hooks/**
if [ $? -ne 0 ]; then
  echo "Error: Failed to set execute permissions on RebeLib hooks."
  exit 1
fi

cp -vrfa scripts/rebelib/hooks/ .git/modules/src/main/java/frc/RebeLib/
if [ $? -ne 0 ]; then
  echo "Error: Failed to copy RebeLib hooks. Please check the scripts/rebelib/hooks directory."
  exit 1
fi

echo ""
echo "RebeLib hooks installed successfully"
