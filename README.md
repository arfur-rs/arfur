![Arfur](./assets/banner.svg)

<h6 align="center">
    <a href="./LICENSE.md">License</a>
  · <a href="https://docs.rs/arfur">Docs</a>
</h6>

<p align="center">
    <a href="https://crates.io/crates/arfur"><img alt="Crates.io" src="https://img.shields.io/crates/v/arfur?color=81B29A&logoColor=D9E0EE&style=for-the-badge"></a>
    <a href="https://docs.rs/arfur"><img alt="docs.rs" src="https://img.shields.io/docsrs/arfur?color=525893&logoColor=D9E0EE&style=for-the-badge"></a>
    <a href="https://codecov.io/gh/arfur-rs/arfur"><img alt="Codecov" src="https://img.shields.io/codecov/c/github/arfur-rs/arfur?color=FFFAEB&style=for-the-badge&token=O04ZY3KQUF"></a>
</p>

**Arfur** is a set of bindings and a framework that builds on top of the [WPILib](https://wpilib.org/) suite, enabling Rust-based robot programs in [FRC](https://www.firstinspires.org/robotics/frc).

<hr/>

```rust
use arfur::prelude::*;

fn main() -> Result<()> {
    let robot: Robot = UninitializedRobot::new().initialize()?;

    // Having a `Robot` type is proof that the HAL has been initialized. We can
    // use to construct all kinds of handles!

    Ok(())
}
```

## Features

 * Rust bindings to WPILib: use WPILib's official C++ implementation
 * Type safety at its finest: strong typing ensures that undefined behaviour *cannot* happen
 * An efficient robot: Stay at C++'s speed, but implicitly use memory-safe concepts as much as possible
 * A powerful ecosystem: hook into Rust's ecosystem for logging, mathematical computations, and more

## Getting started

For now, sift through the crate's [examples](https://github.com/arfur-rs/arfur/tree/main/examples) and [documentation](https://docs.rs/arfur). There's much more to come!
