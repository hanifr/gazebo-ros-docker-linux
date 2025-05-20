#!/bin/bash

echo "🖥️ Setting up XQuartz for Gazebo..."

# Check if XQuartz is installed
if ! command -v xquartz &> /dev/null; then
    echo "❌ XQuartz not found. Please install it using:"
    echo "   brew install --cask xquartz"
    exit 1
fi

# Configure XQuartz for remote connections
defaults write org.xquartz.X11 app_to_run /usr/bin/true
defaults write org.xquartz.X11 no_auth 1
defaults write org.xquartz.X11 nolisten_tcp 0

# Prompt to restart XQuartz
echo "✅ XQuartz configured successfully!"
echo "⚠️ Please restart XQuartz for changes to take effect."
echo "   Then run: xhost + localhost"
echo ""
echo "After restarting XQuartz and running the command above, try:"
echo "   ./scripts/run_gazebo_gui.sh"


