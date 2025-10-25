# `lib/`

These are libraries used by things in the `src/` directory. They're not Colcon packages, and shouldn't be built by `colcon build`.

## Why a monorepo?

[Back in the day](https://github.com/Sooner-Rover-Team/SoonerRoverTeamV), we had a monorepo - and it sucked. So, we understand your concerns more than you could ever know! ;D

However, for a number of reasons, it's valuable in our modern codebase:

- The Rover runs offline when testing, so without a monorepo, we often had trouble building remote dependencies.
- Rust `git` dependencies are useful, but it's harder to make changes to those dependencies.
  - Previously, we'd have to clone the dependency to our computer, make the changes, commit w/ a new version in `Cargo.toml`, and, finally, update the dependency in all our other repos! Not fun...
- `colcon` has no support for building remote dependencies - they need to be in your `colcon` (ROS 2) workspace.
- When doing manual/offline testing on the NVIDIA Jetson, we'd be unable to make changes locally.
  - That means swapping the ethernet cable to one of the bay's WAN-connect ports, then re-opening all your stuff on the new IP, and pulling changes made on GitHub.
  - Local changes are much easier - we can just `scp` them to our computers, then commit there. :D
