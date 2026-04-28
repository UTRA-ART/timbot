#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "=== Pre-Build: Managing Submodules ==="

# 1. Navigate to the submodule
cd sensor_drivers/navsat

# 2. Reset the submodule to its clean, tracked state to prevent patch conflicts
git checkout .
git clean -fd

# 3. Apply your custom patch
echo "Applying patches"
git apply ../../patches/navsat_fix.patch

# 4. Return to the workspace root
cd ../..

echo "=== Building Workspace ==="

echo "=== Post-Build: ZED Open Capture Build ==="
# 10. Build zed_open_source after workspace build
cd sensor_drivers/zed_open_source
mkdir -p build
cd build
cmake ..
make -j$(nproc)

# 11. Return to the workspace root
cd ../../..

# 9. Run your standard build command (add any colcon arguments you usually use here)
colcon build

