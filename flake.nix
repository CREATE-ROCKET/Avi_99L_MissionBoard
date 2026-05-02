{
  description = "Avi 99L Mission Board ESP32-S3 ESP-IDF / esp-hal development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";

    nixpkgs-esp-dev = {
      url = "github:mirrexagon/nixpkgs-esp-dev";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, nixpkgs-esp-dev }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];

      forAllSystems = nixpkgs.lib.genAttrs systems;
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs {
            inherit system;

            overlays = [
              nixpkgs-esp-dev.overlays.default
            ];

            config.permittedInsecurePackages = [
              "python3.13-ecdsa-0.19.1"
            ];
          };

          lib = pkgs.lib;

          espIdfS3 = pkgs.esp-idf-s3-full or pkgs.esp-idf-full;

          espIdfDeps = with pkgs; [
            espIdfS3

            git
            cmake
            ninja
            ccache
            dfu-util
          ] ++ lib.optionals pkgs.stdenv.isLinux [
            usbutils
          ];

          espHalDeps = with pkgs; [
            rustup
            espup
            esp-generate
            espflash
            esptool
            probe-rs-tools
            cargo-make

            git
            pkg-config
            openssl
          ] ++ lib.optionals pkgs.stdenv.isLinux [
            usbutils
          ];
        in
        rec {
          default = pkgs.mkShell {
            name = "avi-99l-esp32s3-idf";

            packages = espIdfDeps;

            shellHook = ''
              export IDF_TARGET=esp32s3

              # まず安全側。必要になったら 1 にする。
              export IDF_CCACHE_ENABLE=0

              # ccache を有効化した場合の保存先。
              export CCACHE_DIR="''${XDG_CACHE_HOME:-$HOME/.cache}/avi-99l/ccache"
              mkdir -p "$CCACHE_DIR"

              echo "ESP32-S3 ESP-IDF shell"
              echo "IDF_PATH=$IDF_PATH"
              echo "IDF_TOOLS_PATH=$IDF_TOOLS_PATH"
              echo "IDF_PYTHON_ENV_PATH=$IDF_PYTHON_ENV_PATH"
              echo "CCACHE_DIR=$CCACHE_DIR"
              echo "Use: idf.py set-target esp32s3 / idf.py build / idf.py flash / idf.py monitor"
            '';
          };

          esp32s3 = default;

          "esp-hal" = pkgs.mkShell {
            name = "avi-99l-esp32s3-esp-hal";

            packages = espHalDeps;

            shellHook = ''
              export IDF_TARGET=esp32s3
              export CARGO_BUILD_TARGET=xtensa-esp32s3-none-elf

              export IDF_PYTHON_CHECK_CONSTRAINTS=no
              export IDF_PYTHON_CHECK_DONE=1

              export RUSTUP_HOME="''${RUSTUP_HOME:-$HOME/.rustup}"
              export CARGO_HOME="''${CARGO_HOME:-$HOME/.cargo}"

              mkdir -p "$RUSTUP_HOME" "$CARGO_HOME"

              # ESP32-S3 は Xtensa なので、通常の stable Rust だけでは足りない。
              # espup が Espressif Rust toolchain と Xtensa 周辺ツールを入れる。
              if ! rustup toolchain list 2>/dev/null | grep -Eq '^esp([[:space:]]|$)'; then
                echo "Espressif Rust toolchain is not installed."
                echo "Running: espup install -t esp32s3"
                espup install -t esp32s3
              fi

              if [ -f "$HOME/export-esp.sh" ]; then
                source "$HOME/export-esp.sh"
              else
                echo "ERROR: $HOME/export-esp.sh was not found."
                echo "Run manually: espup install -t esp32s3"
              fi

              # espup の export 後に再指定する。
              export IDF_TARGET=esp32s3
              export CARGO_BUILD_TARGET=xtensa-esp32s3-none-elf

              # rust-analyzer 用。失敗しても shell 自体は壊さない。
              if command -v rustc >/dev/null 2>&1; then
                rust_sysroot="$(rustc --print sysroot 2>/dev/null || true)"
                if [ -n "$rust_sysroot" ] && [ -d "$rust_sysroot/lib/rustlib/src/rust/library" ]; then
                  export RUST_SRC_PATH="$rust_sysroot/lib/rustlib/src/rust/library"
                fi
              fi

              echo "ESP32-S3 esp-hal shell"
              echo "IDF_TARGET=$IDF_TARGET"
              echo "CARGO_BUILD_TARGET=$CARGO_BUILD_TARGET"
              echo "RUSTUP_HOME=$RUSTUP_HOME"
              echo "CARGO_HOME=$CARGO_HOME"
              echo "probe-rs=$(command -v probe-rs || true)"
              echo "Use:"
              echo "  esp-generate"
              echo "  cargo build"
              echo "  cargo run"
              echo "  espflash flash --monitor <ELF>"
              echo "  probe-rs list"
              echo "  probe-rs info --protocol jtag --chip esp32s3"
              echo "  probe-rs run --protocol jtag --chip esp32s3 <ELF>"
            '';
          };
        });
    };
}