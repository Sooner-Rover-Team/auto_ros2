# `src/` - The ROS 2 Nodes

This folder contains all of our ROS 2 nodes.

## Adding a New Node

To add a new node, you'll need to pick either Python or Rust for the project. Copy the (TODO: some python template) or (TODO: rust template) folder.

### Rust

- Add your new package in the root `/Cargo.toml` in the `members` array.
- Change the name of the `template_node` folder to use the new name.
- Edit the package `Cargo.toml` with `name = "<new_node_name>"`.
- Change the name in `package.xml`.

### Python

- Add your new package in the root `/pyproject.toml` under the `[tool.uv.workspace] members` section.
- Change the node's `pyproject.toml` (`src/<new_python_node_name>/pyproject.toml`) `name` field to the node's new name.
- In `/src/`, rename the `template_node`, `template_node/template_node`, and `template_node/resources/template_node` folders/files, too.
- Do the same in the node's `package.xml`.
