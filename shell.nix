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

    # BINDGEN_EXTRA_CLANG_ARGS = ''                ${builtins.readFile "${stdenv.cc}/nix-support/libc-crt1-cflags"}
    #                                               ${builtins.readFile "${stdenv.cc}/nix-support/libc-cflags"}
    #                                               ${builtins.readFile "${stdenv.cc}/nix-support/cc-cflags"}
    #                                               -idirafter ${stdenv.cc.cc}/lib/clang/${lib.getVersion stdenv.cc.cc}/include
    #                                               -isystem ${stdenv.cc.cc}/include/c++/${lib.getVersion stdenv.cc.cc} -isystem ${stdenv.cc.cc}/include/c++/${lib.getVersion stdenv.cc.cc}/${stdenv.hostPlatform.config} -idirafter ${stdenv.cc.cc}/lib/gcc/${stdenv.hostPlatform.config}/${lib.getVersion stdenv.cc.cc}/include
    #                                               -I ${glibc_multi.dev}/include/ -L ${glibc_multi}/lib
    #                                              '';
  }
