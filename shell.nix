{pkgs ? import <nixpkgs> {}}:
with pkgs;
  (mkShell.override {stdenv = llvmPackages_11.stdenv;}) {
    buildInputs = [
      (pkgs.callPackage ./nix/wpilib-toolchain.nix {})
      pkg-config
      openssl.dev
      cargo-outdated
      cargo-audit
      rust-analyzer
      mdbook
    ];
  }
