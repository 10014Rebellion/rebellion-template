#!/bin/sh
# Installs the pre-commit hook into .git/hooks/

cp scripts/hooks/* .git/hooks/
chmod +x .git/hooks/*
echo "Git hooks installed."
