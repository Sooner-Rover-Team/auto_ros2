# `custom_interfaces_shim`

A workaround for the weird `ament_cargo` dependency graph:

- `ament_cmake` builds the `custom_interfaces` package
- `ament_cargo` builds the `custom_interfaces_shim` package
- Other Rust nodes can then access `custom_interfaces` built because of `custom_interfaces_shim`, and since `custom_interfaces_shim` doesn't depend on the messages being built, it doesn't cause a circular dependency.

**Do not run this node - it has no purpose other than the build.**

## Why?

- Without this shim, `ament_cargo` can't build other Rust packages, as their `Cargo.toml` files can't be parsed due to missing `custom_interfaces` bindings.
- Having it allows `ament_cargo` to build the bindings first, without worrying about dependencies. :D
