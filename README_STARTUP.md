# Multimodal-ckm startup

This repository contains the Version5 FAST-LIVO workspace and its runtime
entrypoints. Runtime logs and sensor recordings are intentionally excluded
from Git.

## Required external environment

- Ubuntu with ROS Noetic installed at `/opt/ros/noetic`.
- The official `livox_ros_driver2` workspace, built at `~/ws_livox` by
  default. Override it with `LIVOX_WS=/path/to/ws_livox`.
- `screen`, `v4l2-ctl`, `iproute2`, `util-linux` (`unshare`, `setpriv`) and
  PCL command-line tools.
- Passwordless sudo for the existing sensor startup operations, or start from
  a shell whose sudo credential is already available.
- The Global Shutter UVC camera and MID360 network interface configured for
  the local machine.
- Gemini 335L is optional. Its ROS workspace defaults to `~/catkin_ws`; use
  `GEMINI_WS=/path/to/workspace`, or start with `START_GEMINI335L=0`.

## First-time setup

Install the stable UVC device rules:

```bash
sudo install -m 0644 system/udev/99-fast-livo-camera.rules \
  /etc/udev/rules.d/99-fast-livo-camera.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Build the repository workspace:

```bash
cd /home/nuc/versions/Multimodal-ckm-main
./build.sh
```

## Start and stop

Start the full stack:

```bash
cd /home/nuc/versions/Multimodal-ckm-main
./start_slam
```

Start without Gemini:

```bash
START_GEMINI335L=0 ./start_slam
```

Stop cleanly and flush PCD/dataset outputs:

```bash
./stop_slam
```

Each run writes diagnostics to `logs/YYYYMMDD_HHMMSS/`. Large images, raw
sensor data and PCD output remain under
`catkin_ws/src/FAST-LIVO2/Log/Multimodal_Dataset/YYYYMMDD_HHMMSS/` and are
ignored by Git.

## Machine-specific defaults

`start_slam` is the stable entrypoint and delegates to the existing
`start_slam1` implementation. The startup currently defaults to:

- MID360 host/interface: `192.168.1.50` on `enp86s0`.
- Stable UVC nodes: `/dev/fast_livo_camera` and
  `/dev/fast_livo_camera_metadata`.
- Livox workspace: `~/ws_livox`.
- Gemini workspace: `~/catkin_ws`.

Override the corresponding environment variables before launch when the host
layout differs.
