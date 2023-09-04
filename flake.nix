{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    parts.url = "github:hercules-ci/flake-parts";

    nci.url = "github:yusdacra/nix-cargo-integration";
    devshell.url = "github:numtide/devshell";
  };
  outputs = {
    self,
    parts,
    nci,
    devshell,
    ...
  } @ inputs:
    parts.lib.mkFlake {inherit inputs;} {
      systems = ["x86_64-linux"];

      imports = [
        nci.flakeModule
        devshell.flakeModule
      ];

      perSystem = {
        config,
        pkgs,
        ...
      }: {
        nci.projects.arfur.relPath = "";

        devShells.default = config.nci.outputs.arfur.devShell.overrideAttrs (rust: {
          packages = with pkgs; [
            (pkgs.callPackage ./nix/wpilib-toolchain.nix {})

            pkg-config
            openssl.dev
            cargo-outdated
            cargo-audit
            cargo-release
            cargo-edit
            cargo-expand
            git-cliff
            glibc_multi
            rust-analyzer
          ];

          env."LIBCLANG_PATH" = "${pkgs.libclang.lib}/lib";
          env."BINDGEN_EXTRA_CLANG_ARGS" = with pkgs; ''
            ${builtins.readFile "${stdenv.cc}/nix-support/libc-crt1-cflags"}
            ${builtins.readFile "${stdenv.cc}/nix-support/libc-cflags"}
            ${builtins.readFile "${stdenv.cc}/nix-support/cc-cflags"}
            ${builtins.readFile "${stdenv.cc}/nix-support/libcxx-cxxflags"}

            ${lib.optionalString stdenv.cc.isClang "-idirafter ${stdenv.cc.cc}/lib/clang/${lib.getVersion stdenv.cc.cc}/include"}
            ${lib.optionalString stdenv.cc.isGNU "-isystem ${stdenv.cc.cc}/include/c++/${lib.getVersion stdenv.cc.cc} -isystem ${stdenv.cc.cc}/include/c++/${lib.getVersion stdenv.cc.cc}/${stdenv.hostPlatform.config} -idirafter ${stdenv.cc.cc}/lib/gcc/${stdenv.hostPlatform.config}/${lib.getVersion stdenv.cc.cc}/include"}
            -I ${glibc_multi.dev}/include/ -L ${glibc_multi}/lib
          '';
          env."TARGET_CXX" = "arm-frc2023-linux-gnueabi-g++";
        });
      };
    };
}
