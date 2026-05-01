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
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ nixpkgs-esp-dev.overlays.default ];

            config.permittedInsecurePackages = [
              "python3.13-ecdsa-0.19.1"
            ];
          };
        in
        {
          default = pkgs.mkShell {
            name = "esp32s3-idf";

            buildInputs = with pkgs; [
              esp-idf-xtensa
              git
              cmake
              ninja
              ccache
              dfu-util
            ] ++ nixpkgs.lib.optionals pkgs.stdenv.isLinux [
              usbutils
            ];

            shellHook = ''
              export IDF_TARGET=esp32s3
              export IDF_CCACHE_ENABLE=1
              echo "ESP32-S3 ESP-IDF shell"
              echo "Use: idf.py build / idf.py flash / idf.py monitor"
            '';
          };
        });
    };
}
