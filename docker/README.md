# Timbot sim in Docker + Foxglove

Runs the full sim (Gazebo, EKFs, Cartographer, Nav2) headless in a container on
Windows, macOS or Linux, and shows it in Foxglove instead of RViz/Gazebo windows.
No ROS install or GPU needed on your machine.

## Setup (once)
1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
   (Linux: Docker Engine + the compose plugin also works).
2. Docker Desktop → Settings → Resources: give it **at least 4 GB memory and 4 CPUs**.
3. **Windows only**, before cloning: `git config --global core.autocrlf input`.
   Git for Windows otherwise checks scripts out with CRLF line endings, which break
   them inside the Linux container (the repo's `.gitattributes` can't cover the
   submodules). Or clone from a WSL (Ubuntu) shell instead and run `docker compose`
   there; both work and the dev build speed is similar.
4. Clone the repo with submodules:
   `git clone --recurse-submodules https://github.com/UTRA-ART/timbot.git`
5. Get access to the UTRA ART workspace on [Foxglove](https://app.foxglove.dev).

## Run the sim
From the repo root:
```bash
docker compose up
```
This pulls the prebuilt image (`ghcr.io/utra-art/timbot-sim`, built from `main`
by CI) and starts the sim. Stop it with `Ctrl+C`. To get the latest image: `docker compose pull`.
If the image isn't published (yet), you'll see `error from registry: denied` (Docker
Desktop: `unauthorized`) and it
builds locally instead (about 5–20 min, once). That's expected.

Then open Foxglove → **Connect to a robot** → **Foxglove WebSocket** → `ws://localhost:8765`,
or go straight to
`https://app.foxglove.dev/utra-art/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765`.
A starter layout is in `docker/foxglove_layout.json` (layout menu → **Import from file**).

## Develop (run your own code)
```bash
docker compose up dev
```
Mounts your checkout into the container and runs `colcon build` on every start,
so after editing code just stop and rerun it (a few seconds when little changed).
The first run builds a local image (10–20 min, needs internet). After that it works offline.

- Build output lives in Docker volumes, not your checkout. If the build gets into
  a bad state: `docker compose down -v`.
- Changed a `package.xml` or the Dockerfile? Rebuild the image: `docker compose build dev`.
- Shell inside the running sim: `docker compose exec dev bash`, then
  `source /ws/install/setup.bash` and use `ros2 topic list` etc. as usual.

## Options
Set before the command, e.g. `TIMBOT_CV=1 docker compose up dev`
(PowerShell: `$env:TIMBOT_CV=1; docker compose up dev`).

| Variable | Effect |
|---|---|
| `TIMBOT_CV=1` | Simulated ZED camera + lane/depth detection (`/zed_node/left/image`, `/cv/model_output`). Heavy without a GPU. |
| `TIMBOT_CONFIG=<file>` | Launch config from `launch/config/` (default `sim_headless.yaml`: `sim.yaml` minus GUI/RViz). |
| `TIMBOT_TAG=<tag>` | Which published image `docker compose up` pulls (default `latest`, built from `main`; `docker` = the `docker` branch). |

## Using it in Foxglove
- **Send a nav goal:** in the 3D panel, right-click the publish button (⊕, right-hand
  toolbar) → *Pose*, then click it and click-drag on the map (click = position,
  drag = heading). Like RViz's 2D Goal Pose. Works with Foxglove's default pose topic
  (`/move_base_simple/goal`, relayed in the container) or `/goal_pose`.
- **Drive manually:** a Teleop panel on `/key_vel` (overrides Nav2 via twist_mux).
- **Camera:** an Image panel on `/zed_node/left/image` (needs `TIMBOT_CV=1`).

## Linux + NVIDIA GPU (optional)
Not needed for the default sim, which runs at full speed on the CPU. The GPU
halves Gazebo's CPU use and makes `TIMBOT_CV=1` cheap. Install the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), then:
```bash
xhost +SI:localuser:root    # once per login: let the container use your X display
docker compose -f compose.yaml -f docker/compose.gpu.yaml up        # or: ... up dev
```
No window opens; Gazebo just uses your display's GPU to render its sensors.

## Troubleshooting
- **`exec /start_sim.sh: no such file or directory`, or colcon fails with
  `Syntax error: end of file unexpected` (Windows):** the clone has CRLF line
  endings. Run `git config --global core.autocrlf input`, delete the clone, clone it
  again, then `docker compose build` (and `docker compose down -v` for dev mode).
- **Foxglove can't connect / sim has no sensor data, with a VPN on:** Cisco Secure
  Client (AnyConnect) blocks traffic to Docker's networks while connected. Disconnect
  it, or enable *Allow local (LAN) access* in its preferences if your profile allows.

## Limits
- Simulation only. The real rover (`comp.yaml`, serial/USB devices, the Pi link) stays on the native install.
- CV uses the classical lane detector; torch/YOLO isn't in the image.
- The Gazebo GUI and RViz don't run here; Foxglove replaces both.

## Maintainers: the published image
`.github/workflows/docker.yml` builds `--target sim` for amd64 and arm64 on native
runners and pushes `ghcr.io/utra-art/timbot-sim`:
`:latest` and `:main` on pushes to `main`, `:docker` on pushes to the `docker`
branch (temporary, while this is being trialled), `:<tag>` for `v*` tags, and
`:<branch>` for manual runs (Actions → Docker image → Run workflow; only once the
workflow is on `main`). Pull requests that touch the Docker files or any
`package.xml` are built but not pushed.

After the first publish, make the package public (GitHub → UTRA-ART → Packages →
timbot-sim → Package settings → Change visibility), or members will need
`docker login ghcr.io` to pull it.
