#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <submodule_path>"
    exit 1
fi

if [ ! -d ./gradle ]; then
    echo "Error: Gradle directory does not exist. Please run this script from the root of the project."
    exit 1
fi

if [ -d "$1" ]; then
    echo "Submodule path provided: $1"
else
    echo "Error: Submodule path does not exist or is not a directory."
    exit 1
fi

PROJECT_ROOT=$(pwd)
# ./update-submodule.sh <submodule_path>
SUBMODULE=$1

# check for local updates to submodule
cd $SUBMODULE
if [ -z "$(git status --porcelain)" ]; then
    if [ -z "$(git log @{u}..)" ]; then
        echo "Submodule is up to date."
    else
        echo "Submodule has un-pushed changes."
        exit 1
    fi
else 
    echo "Submodule has uncommitted changes."
    exit 1
fi

# check if submodule is on a non-main branch
CURRENT_HASH=$(git rev-parse HEAD)
git fetch origin
git checkout main > /dev/null 2>&1
if [ CURRENT_HASH != "$(git rev-parse HEAD)"  ]; then
    echo "Submodule is on a non-main branch, allowing update. CI will not allow merge until this is resolved."
    git checkout $CURRENT_HASH > /dev/null 2>&1
    exit 0
fi

cd "$PROJECT_ROOT"

# check for remote updates to submodule
git add $SUBMODULE
BEFORE_UPDATE=$(git submodule status)
git submodule update --init --recursive --remote --merge

if [ "$BEFORE_UPDATE" != "$(git submodule status)" ]; then
    echo "Submodule is not the latest version."
    exit 1
else
    echo "No changes in submodules."
    exit 0
fi
