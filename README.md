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
git clone --recurse-submodules --remote-submodules <the URL of this repository>.git
cd <the name of the repository>
./scripts/install-hooks.sh
```

Then open the root directory of the repository in WPILib VSCode

## Flow

All code changes should happen on a branch, and then a PR should be opened and reviewed by multiple senior members or leads before being merged. Commits should contain a single set of related changes, and commit names should be descriptive of their contents. Err on the side of granularity when committing, as it's easier to clean up later. Push whenever code is in a reasonable state and prior to leaving. Pull often, and keep up with changes on `main` and other branches. Reviews should be thorough, lest you tear your hair out debugging later (do *not* LGTM). No code should go into `main` that hasn't been simulated thoroughly and/or tested on a robot, and no incomplete or in-progress code should be merged. All code should be formatted on build, and should follow WPILib command-based programming best practices.

## RebeLib

RebeLib exists within this project as a git submodule, managed by a set of scripts. To make changes to RebeLib, change into the `src/main/java/frc/RebeLib` directory and branch, commit, and push your changes. Once your RebeLib changes are pushed to a branch, you can commit and push to this project. A Github Action requires that RebeLib be set to the newest `main` before merging, so any PRs to RebeLib must be merged first. Once RebeLib is merged, pull it from this project and commit the new hash to make the Action pass so that you can merge.

Any code that is season-agnostic should live in RebeLib, and should be maintained.

## Github Pages Javadoc

A Github Action will automatically generate Javadocs for the robot project, which will live on the `javadocs` branch and be posted to the project's Github Pages site
