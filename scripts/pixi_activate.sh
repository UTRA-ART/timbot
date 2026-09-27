#!/bin/bash
# Sourced automatically by pixi on `pixi shell` / `pixi run` (see [activation]
# in pixi.toml). Only sources the local colcon overlay if it exists yet, so a
# fresh clone that hasn't been built doesn't break activation.
if [ -f "$PIXI_PROJECT_ROOT/install/setup.bash" ]; then
    source "$PIXI_PROJECT_ROOT/install/setup.bash"
fi
