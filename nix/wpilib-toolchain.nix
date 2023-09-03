{
  config,
  lib,
  pkgs,
  ...
}:
pkgs.stdenv.mkDerivation rec {
  name = "wpilib-cross-compiler";
  version = "2023-9";

  nativeBuildInputs = with pkgs; [
    # Patch our binaries!
    autoPatchelfHook

    # Binary dependencies (patched during build)
    # TODO: make sure we actually need each of these deps
    ncurses5.dev
    zlib.dev
    expat.dev
    xz.dev
    python3
    libclang.dev
    mpfr.dev
  ];

  src = builtins.fetchTarball {
    url = "https://github.com/wpilibsuite/opensdk/releases/download/v2023-9/cortexa9_vfpv3-roborio-academic-2023-x86_64-linux-gnu-Toolchain-12.1.0.tgz";
    sha256 = "0h7c1qc0jmw3a3jb1v1d40ld6yz0fr65pn9xjxb51f8zm0q3k02l";
  };

  installPhase = ''
    mkdir $out
    cp -r . $out
  '';
}
