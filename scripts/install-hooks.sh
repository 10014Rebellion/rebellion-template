#!/bin/sh

cp scripts/hooks/* .git/hooks/
chmod +x .git/hooks/*
echo "Git hooks installed."
