# HighStakesAlpha24

Welcome to the main software repository for KUdos VEX-U High Stakes season (2024-2025 Alpha Robot). This project has all driver control, autonomous routines, and custom sub-system logic built on the PROS framework and EZ-Template.

## Architecture & Features

* Odometry & Localization: Configured for dual-pod position tracking and custom AprilTag/vision pipelines.
* Autonomous Selector: Integrated LCD menu to easily select dynamic auto paths before matches.
* Mechanism Logic: Custom PID loops and controls for the intake, end effectors, and lifts.

## Tech Stack

* Framework: PROS Kernel
* Motion Library: EZ-Template (v3.2.0-rc.1)
* Language: C++17

## Getting Started

To work on this codebase, make sure you have the PROS CLI installed on your machine.

1. Clone the repository to your local workspace.
2. Open your terminal in the root directory.
3. Build and test your changes before pushing.

## Quick CLI Commands

Build the codebase:
pros make

Flash code to the V5 Brain:
pros upload

Build and flash at the same time:
pros mu

Open serial stream for debugging:
pros terminal

## Contribution Guidelines

* Format your code before committing using the included .clang-format rules.
* Test all autonomous routines on the physical test rig or chassis before opening a pull request.
* Keep commit messages clear and descriptive so the whole team can follow along.
