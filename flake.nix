{
  inputs = {
    nci.url = "github:yusdacra/nix-cargo-integration";
  };
  outputs = {
    self,
    nci,
    ...
  } @ inputs:
    nci.lib.makeOutputs {
      root = ./.;

      overrides.shell = common: prev: {
        # startup = prev.startup // {
        #   setupStdenv.text = "${common.pkgs.llvmPackages.stdenv}/setup";
        # };

        packages =
          prev.packages
          ++ (with common.pkgs; [
            (pkgs.callPackage ./nix/wpilib-toolchain.nix {})
            pkg-config
            openssl.dev
            cargo-outdated
            cargo-audit
            git-cliff
            glibc_multi
            rust-analyzer
            mdbook
          ]);

        env =
          prev.env
          ++ [
            {
              name = "LIBCLANG_PATH";
              eval = "${common.pkgs.libclang.lib}/lib";
            }
            {
              name = "BINDGEN_EXTRA_CLANG_ARGS";
              eval = with common.pkgs; ''                "$(< ${stdenv.cc}/nix-support/libc-crt1-cflags) \
                                                          $(< ${stdenv.cc}/nix-support/libc-cflags) \
                                                          $(< ${stdenv.cc}/nix-support/cc-cflags) \
                                                          $(< ${stdenv.cc}/nix-support/libcxx-cxxflags) \
                                                          ${lib.optionalString stdenv.cc.isClang "-idirafter ${stdenv.cc.cc}/lib/clang/${lib.getVersion stdenv.cc.cc}/include"} \
                                                          ${lib.optionalString stdenv.cc.isGNU "-isystem ${stdenv.cc.cc}/include/c++/${lib.getVersion stdenv.cc.cc} -isystem ${stdenv.cc.cc}/include/c++/${lib.getVersion stdenv.cc.cc}/${stdenv.hostPlatform.config} -idirafter ${stdenv.cc.cc}/lib/gcc/${stdenv.hostPlatform.config}/${lib.getVersion stdenv.cc.cc}/include"} \
                                                          -I ${glibc_multi.dev}/include/ -L ${glibc_multi}/lib
                                                        "'';
            }
          ];
      };
    };
}
