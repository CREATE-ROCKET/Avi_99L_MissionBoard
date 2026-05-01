{
  description = "ESP32-S3 ESP-IDF project";

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
          lib = nixpkgs.lib;

          pkgs = import nixpkgs {
            inherit system;
            overlays = [ nixpkgs-esp-dev.overlays.default ];

            config.permittedInsecurePackages = [
              "python3.13-ecdsa-0.19.1"
            ];
          };

          espIdf = pkgs.esp-idf-full;

          # ESP-IDF の idf_tools.py export は
          # $IDF_TOOLS_PATH/tools/<tool-name>/<version>
          # という通常インストール構造を期待する。
          #
          # nixpkgs-esp-dev の各 tool derivation をその形に symlink して、
          # VSCode 拡張や idf_tools.py export が落ちないようにする。
          idfToolsPath = pkgs.runCommand "esp-idf-tools-path" {
            nativeBuildInputs = [ pkgs.python3 ];

            toolMap =
              builtins.toJSON
                (lib.mapAttrs (_: drv: "${drv}") espIdf.tools);
          } ''
            mkdir -p "$out/tools" "$out/dist"

            python3 - <<'PY'
import json
import os

tool_map = json.loads(os.environ["toolMap"])

with open("${espIdf}/tools/tools.json", "r", encoding="utf-8") as f:
    tool_specs = json.load(f)["tools"]

versions = {
    tool["name"]: tool["versions"][0]["name"]
    for tool in tool_specs
}

for name, store_path in tool_map.items():
    version = versions.get(name)
    if version is None:
        raise SystemExit(f"{name} is not listed in ESP-IDF tools.json")

    tool_dir = os.path.join(os.environ["out"], "tools", name)
    os.makedirs(tool_dir, exist_ok=True)
    os.symlink(store_path, os.path.join(tool_dir, version))
PY
          '';
        in
        rec {
          default = pkgs.mkShell {
            name = "avi-99l-esp32s3-idf";

            buildInputs = with pkgs; [
              espIdf

              git
              cmake
              ninja
              ccache
              dfu-util
            ] ++ lib.optionals pkgs.stdenv.isLinux [
              usbutils
            ];

            shellHook = ''
              export IDF_TARGET=esp32s3
              export IDF_CCACHE_ENABLE=1

              # idf_tools.py export 対策。
              export IDF_TOOLS_PATH=${idfToolsPath}

              # custom installation 扱いでも ESP-IDF 側がバージョン判定できるようにする。
              export ESP_IDF_VERSION="$(cat "$IDF_PATH/version.txt" | sed 's/^v//' | cut -d. -f1,2)"

              echo "ESP32-S3 ESP-IDF shell"
              echo "IDF_PATH=$IDF_PATH"
              echo "IDF_TOOLS_PATH=$IDF_TOOLS_PATH"
              echo "Use: idf.py set-target esp32s3 / idf.py build / idf.py flash / idf.py monitor"
            '';
          };

          esp32s3 = default;
        });
    };
}
