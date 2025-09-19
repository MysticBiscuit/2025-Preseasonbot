#!/usr/bin/env bash
set -euo pipefail

# Get a WPILib 2025 VSIX into /opt and install it by path via devcontainer settings.
# If you prefer a specific point release, pin the URL below to that tag.
VSIX_URL="${WPILIB_VSIX_URL:-https://github.com/wpilibsuite/vscode-wpilib/releases/latest/download/vscode-wpilib-2025.0.0.vsix}"

sudo mkdir -p /opt
sudo rm -f /opt/vscode-wpilib-2025.vsix
echo "Downloading WPILib VSIX from: $VSIX_URL"
sudo curl -fsSL "$VSIX_URL" -o /opt/vscode-wpilib-2025.vsix

# Warm Gradle caches for faster first builds
./gradlew --no-daemon help || true
