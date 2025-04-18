# Contributing

This document contains information about contributing to the project. Please read through if you're ready to start working with us!

## Setup

You'll need a few things to continue.

- [`git`](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git): used to manage the codebase
- [`uv`](https://docs.astral.sh/uv/getting-started/installation/): manages Python, a scripting language
- [Rust](https://rustup.rs/): the second language we use
- A code editor, like [Zed Editor](https://zed.dev/download) or [Visual Studio Code](https://code.visualstudio.com/).
- [A GitHub account](https://github.com/login): to see tasks and contribute your ideas

Each of those links will guide you though setting it up. If you need any help, please reach out!

## Git

We use `git` and GitHub to manage our code and track progress on our goals.

Please [install `git`](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) on your computer to make sure everything works right! You may also want the GitHub CLI (`gh` command) to log in to GitHub, and easily checkout pull requests. We'll mention those later. But, for now, [please make sure to install it](https://cli.github.com/)!

### Commits

In git terminology, ["commits" are logical changes to a project](https://www.freecodecamp.org/news/git-commit-command-explained/). We make lots of them when developing the Rover's code! Please make sure to ["commit early and often"](https://sethrobertson.github.io/GitBestPractices/#commit) - it'll make reviewing your code much easier! :)

On Rover, we use the [Conventional Commits style guide](https://www.conventionalcommits.org/en/v1.0.0/) for commit messages. It helps folks know what you're changing. Here are a few examples of good commit messages: 🔥🦾

- `fix(interfaces/WheelsMsg): typo`
- `feat(nav): improve path planning`
- `docs(README): link to launch file tutorial`

And some bad commit messages: 😭❌

- `typo`
- `Fixed the Navigator Node's path planning by adding a parameter to the ROS 2 node which sho...`
- `linked tutorial`

### Pull Requests

We use pull requests (PRs) to separate our work. We've got many members, so we don't usually push to the `main` branch.

For more info about how to use pull requests, please [see this short article](https://foundations.projectpythia.org/foundations/github/github-workflows.html). You'll often want to download someone's PR onto your machine - the process of [doing this is explained here](https://www.jesusamieiro.com/using-the-github-cli-to-review-pr/).

### Review

We review each other's pull requests to ensure high-quality work. It helps catch bugs. [This article from Project Pythia](https://foundations.projectpythia.org/foundations/github/review-pr.html) is great at explaining why and how to do it!

### CI

We have [automatic checks that run](https://en.wikipedia.org/wiki/Continuous_integration) when you make a pull request or push new code. The CI system will "continously integrate" new changes, building and testing them. After they pass, and the review process is complete, we'll merge your code into `main`!

### Merge Commits

We've used merge commits before, but we tend to "rebase merge" submitted PRs to keep the commit history clean. This avoids potentially messy and incompatible histories, and helps the repo stay accessible to new members.

## Code Style

- Documentation
  - Please document everything you make!
    - Python has `"""` doc-strings, and Rust has `///` doc-comments.
  - Each node/package should have its own `README.md` file.
- Testing
  - Write tests for the code you make.
    - Rust has [in-module testing and external testing](https://doc.rust-lang.org/book/ch11-03-test-organization.html), while Python only has external [testing through Pytest](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html).
  - Document the reasons behind tests.
    - It won't be clear in several months - we always forget lol
    - Adding a doc-string/comment can help a lot!
- Typing
  - A lot of people neglect types in Python, but we use `basedpyright` and `ruff` to ensure correct type usage.
  - Always use the type syntax (like `speed: float = 0.0`) in your functions and types.
  - We check these in CI - we can't merge your PR until it's typed properly.
