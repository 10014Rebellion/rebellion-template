# Rebellion Template

[![Build and Test](https://github.com/10014Rebellion/rebellion-template/actions/workflows/build.yml/badge.svg)](https://github.com/10014Rebellion/rebellion-template/actions/workflows/build.yml)
<!-- //TODO Update this URL so it points to the right repo -->

## Setting up the environment

### Requirements

- git (w/ git bash on Windows)
- WPILib

### Installing

Open git bash or your sh of choice and run the following commands:

```sh
git clone <the URL of this repository>.git
cd <the name of the repository>
./scripts/install-hooks.sh
```

Then open the root directory of the repository in WPILib VSCode

## RebeLib

RebeLib exists within this project as a git submodule, managed by a set of scripts. To make changes to RebeLib, change into the `src/main/java/frc/RebeLib` directory and branch, commit, and push your changes. Once your RebeLib changes are pushed to a branch, you can commit and push to this project. A Github Action requires that RebeLib be set to the newest `main` before merging, so any PRs to RebeLib must be merged first. Once RebeLib is merged, pull it from this project and commit the new hash to make the Action pass so that you can merge.

Any code that is season-agnostic should live in RebeLib, and should be maintained.

## Github Pages Javadoc

A Github Action will automatically generate Javadocs for the robot project, which will live on the `javadocs` branch and be posted to the project's Github Pages site
