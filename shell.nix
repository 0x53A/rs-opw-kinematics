{ pkgs ? import <nixpkgs> {} }:

let
  libPath = with pkgs; lib.makeLibraryPath [
    libGL
    libxkbcommon
    wayland
    vulkan-loader
    alsa-lib
  ];
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    pkg-config
    libxkbcommon
    wayland
    vulkan-headers
    vulkan-loader
    alsa-lib
    cmake
  ];

  LD_LIBRARY_PATH = libPath;
}
