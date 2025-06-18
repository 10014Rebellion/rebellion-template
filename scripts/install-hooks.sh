#!/bin/sh
# Installs the pre-commit hook into .git/hooks/

HOOK_SRC="scripts/hooks/pre-commit"
HOOK_DEST=".git/hooks/pre-commit"

cp "$HOOK_SRC" "$HOOK_DEST"
chmod +x "$HOOK_DEST"

echo "Pre-commit hook installed."
