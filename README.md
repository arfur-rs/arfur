![Arfur](./assets/banner.svg)

<h6 align="center">
    <a href="./LICENSE.md">License</a>
  · <a href="https://docs.rs/arfur">Docs</a>
</h6>

<p align="center">
    <a href="https://crates.io/crates/arfur"><img alt="Crates.io" src="https://img.shields.io/crates/v/arfur?color=81B29A&logoColor=D9E0EE&style=for-the-badge"></a>
    <a href="https://docs.rs/arfur"><img alt="docs.rs" src="https://img.shields.io/docsrs/arfur?color=525893&logoColor=D9E0EE&style=for-the-badge"></a>
</p>

**Arfur** is a set of bindings and a framework that builds on top of the [WPILib](https://wpilib.org/) suite, enabling Rust-based robot programs in [FRC](https://www.firstinspires.org/robotics/frc).

<hr/>

```rust
use arfur::prelude::*;

fn main() -> Result<()> {
    let robot: Robot = RobotBuilder::default().initialize()?;

    // Having a `Robot` type is proof that the HAL has been initialized. We can
    // use to construct all kinds of handles!

    Ok(())
}
```

## Features

 * Rust bindings to WPILib: Arfur is a set of bindings, not a reimplementation,
   meaning you can expect the reliability of the official WPILib C++
   implementation.

 * Type safety at its finest: The public API is idiomatically Rust, meaning you
   have type-level gaurantees that undefined behaviour will not and *cannot*
   occur.

 * An efficient robot: Keep the runtime speed provided by C++. Arfur is low- to
   zero- overhead.

 * A powerful ecosystem: Hook into Rust's ecosystem for logging, mathematical
   computations, and more!

## Requirements

What do you need to get started?

<!-- TODO: getting started guide that outlines toolchain installation -->

 * **Rust**
   * MSRV: `1.72.0`
   * Toolchain: `stable`
 * **ARM Compiler Toolchain**: Since you're building to the RoboRIO, which uses
   a different architecture than most personal computers, you'll need a
   toolchain that can build to ARM.
   * Option 1: install a generic ARM toolchain.
   * Option 2: install the
     [FRC ARM toolchain](https://github.com/wpilibsuite/opensdk/).
     * MSRV: `2023-9`
   * Don't forget to point Arfur to your toolchain executable.
 * (optional) **Nix**: Use [Nix](https://nixos.org/) to ditch both requirements
   mentioned above.
   * Your choice of Rust installation method.
   * Use the Nix derivation in `./nix` to grab the FRC ARM toolchain.

<!-- TODO: Make Nix drv public -->

## Getting started

Arfur is still highly in development. Do not consider this project stable by
any means.

A complete guide is, unfortunately, still in the works. For now, sift through
the crate's [examples](https://github.com/arfur-rs/arfur/tree/main/examples)
and [documentation](https://docs.rs/arfur). There's more to come!

## Development

So, you want to help out with this project? We'd love to chat about the
project, just open an issue or make a PR.

Due to the inherently complex nature of generate bindings for large projects,
Arfur uses `xtask` to manage the project. After getting access to the source
code (presumably via cloning the git repository), run:

```
cargo xtask bindgen
```

This will generate the bindings as necessary. Then, run:

```
cargo b --example raw_usage --target arm-unknown-linux-gnueabi
```

To build an example.
