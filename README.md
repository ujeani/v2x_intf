# v2x_intf_pkg

ROS 2 bridge between `v2x_intf_msg/Recognition` topics and a UDP V2X endpoint.

## Architecture

The runtime has three layers:

- `transport/udp.py`: nonblocking UDP I/O only
- `protocol/`: packet, recognition, and optional WAVE IFM codecs
- `ros/bridge_node.py`: ROS subscription, publication, and receive timer

Outgoing messages flow from `v2x/recognition` through `RecognitionCodec` to UDP.
Incoming UDP packets are decoded and published on `v2x/r_recognition`.

## Build with colcon

This repository is a ROS 2 package and should be placed in the `src` directory
of a colcon workspace. The `v2x_intf_msg` package must also be installed or
available in the same workspace.

Open Bash and source your ROS 2 installation. Replace `<distro>` with the ROS 2
distribution installed on the computer, for example `humble` or `jazzy`.

```bash
source /opt/ros/<distro>/setup.bash
```

From the workspace root (the directory containing `src`), install dependencies
and build the package:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select v2x_intf_pkg
```

`--symlink-install` is useful during Python development because many source
changes become available without copying the package again. Rebuild after
changing package metadata, entry points, or installed resources.

After a successful build, source the workspace overlay:

```bash
source install/setup.bash
```

The overlay must be sourced in every new Bash terminal. To source it
automatically, add it to `~/.bashrc`:

```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Run from Bash

Start the bridge with its default configuration:

```bash
ros2 run v2x_intf_pkg v2x_intf_node
```

To select the UDP endpoints explicitly:

```bash
ros2 run v2x_intf_pkg v2x_intf_node \
  --obu-ip 127.0.0.1 \
  --obu-port 1516 \
  --local-port 5398
```

The arguments mean:

- `--obu-ip`: destination OBU or V2X service IP address
- `--obu-port`: destination UDP port used for outgoing packets
- `--local-port`: local UDP port on which incoming packets are received

The defaults are defined in `v2x_intf_pkg/config.py`. Stop the node with
`Ctrl+C`.

## Verify the ROS topics

In another Bash terminal, source the same ROS installation and workspace, then
inspect the topics:

```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /v2x/r_recognition
```

Outgoing `v2x_intf_msg/Recognition` messages should be published to
`/v2x/recognition`. Valid incoming UDP packets are decoded and published on
`/v2x/r_recognition`.

If the node reports that the local address is already in use, stop the process
using that UDP port or run the bridge with a different `--local-port` value.
