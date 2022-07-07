{
  description = "UTP TM4C Flake";

  inputs = {
    nixpkgs.url      = github:NixOS/nixpkgs/nixos-21.11;
    rust-overlay.url = github:oxalica/rust-overlay;
    flake-utils.url  = github:numtide/flake-utils;
    nur.url          = github:nix-community/NUR;
    # TODO: ditch this once this PR is merged: https://github.com/NixOS/nixpkgs/pull/175052
    nixpkgs-with-lm4tools.url = github:rrbutani/nixpkgs/feature/lm4tools;
  };

  # TODO: cargo extensions (cargo-expand)
  # TODO: CI setup? (garnix)
  # TODO: expose targets, etc.
  # TODO: flip-link, probe-run, tests, etc.

  outputs = { self, nixpkgs, rust-overlay, flake-utils, nur, nixpkgs-with-lm4tools }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        # TODO: make a nixpkg of its own and upstream:
        gdb-tools = pkgs: with pkgs.python3Packages; buildPythonPackage rec {
          pname = "gdb-tools";
          version = "1.4";

          propagatedBuildInputs = [ arpeggio ];
          src = fetchPypi {
            inherit pname version;
            sha256 = "NYtmI+0qeVx58vy49CRMEZw1jzZOgwElHUVIE1VpNEc=";
          };

          format = "pyproject";

          meta = with pkgs.lib; {
            # maintainers = with maintainers; [ TODO ];
            # description = "TODO"
            # license = bsd clause 3
          };
        };

        # TODO: upstream:
        flip-link = pkgs: with pkgs; rustPlatform.buildRustPackage rec {
          pname = "flip-link";
          version = "0.1.6";

          src = fetchFromGitHub {
            owner = "knurling-rs";
            repo = pname;
            rev = "v${version}";
            sha256 = "sha256-Sf2HlAfPlg8Er2g17AnRmUkvRhTw5AVPuL2B92hFvpA=";
          };

          # TODO: fix?
          doCheck = false;

          cargoSha256 = "sha256-2VgsO2hUIvSPNQhR13+bGTxXa6xZXcK0amfiWv2EIxk=";
          buildInputs = lib.optional stdenv.isDarwin libiconv;
        };

        overlays = [ (import rust-overlay) nur.overlay ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        lm4tools = (import nixpkgs-with-lm4tools {
          inherit system;
        }).lm4tools;

        # `gdb` is broken on ARM macOS so we'll fallback to using x86_64 GDB
        # there (assuming Rosetta is installed: https://github.com/NixOS/nix/pull/4310).
        #
        # See: https://github.com/NixOS/nixpkgs/issues/147953
        gdbPkgs' = let
          pkgs' = if pkgs.stdenv.isDarwin && pkgs.stdenv.isAarch64 then
            (import nixpkgs { system = "x86_64-darwin"; inherit overlays; })
          else
            pkgs;
        in
          [ pkgs'.gdb pkgs'.nur.repos.mic92.gdb-dashboard (gdb-tools pkgs') ]
        ;

        # As per https://github.com/ut-utp/.github/wiki/Dev-Environment-Setup#embedded-development-setup
        # on Linux we need to expose `gdb` as `gdb-multiarch`
        # (to match other distros):
        gdbPkgs = if pkgs.stdenv.isLinux then
          let
            baseGdb = builtins.head gdbPkgs';
            gdbMultiarch = pkgs.stdenvNoCC.mkDerivation {
              pname = "gdb-multiarch";
              inherit (baseGdb) version meta;
              nativeBuildInputs = with pkgs; [ makeWrapper ];
              unpackPhase = "true";
              installPhase = ''
                mkdir -p $out/bin
                makeWrapper ${baseGdb}/bin/gdb $out/bin/gdb-multiarch
              '';
            };
          in
          [gdbMultiarch] ++ gdbPkgs'
        else
          gdbPkgs';

        tm4c-svd-file = pkgs.fetchurl {
          url = "https://raw.githubusercontent.com/posborne/cmsis-svd/551849db7be8415b1acc19bcc7fbbb07e808a4bf/data/TexasInstruments/TM4C123GH6PM.svd";
          sha256 = "Vi0SL/0vmefGrsBzOE/comH+8PiUJOyd1wF03kaDf6o=";
        };

        rust-toolchain = (pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml);
        llvm-tools-preview = builtins.head (builtins.filter (p: p.pname == "llvm-tools-preview") rust-toolchain.paths);
      in
      with pkgs;
      {
        devShells.default = mkShell {
          buildInputs = [
            rust-toolchain
            openocd

            cargo-bloat cargo-asm cargo-expand
            (flip-link pkgs)
            lm4tools
            picocom
          ] ++ gdbPkgs ++ [
            # For `xtask`:
            pkg-config # host serial stuff needs this
            openssl    # reqwest needs this
          ] ++ lib.optionals (pkgs.stdenv.isLinux) [
            libudev    # host serial stuff, again
          ];
          shellHook = ''
            LLVM_TOOLS_PREVIEW_BIN=$(echo ${llvm-tools-preview}/lib/rustlib/*/bin)
            export PATH="$PATH:$LLVM_TOOLS_PREVIEW_BIN"

            if [[ -d .vscode ]] && [[ ! -f .vscode/TM4C123GH6PM.svd ]]; then
              ln -s ${tm4c-svd-file} .vscode/TM4C123GH6PM.svd
            fi
          '';
        };
      }
    );
}
