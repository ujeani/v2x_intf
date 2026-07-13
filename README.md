# v2x_intf_pkg

ROS 2 bridge between `v2x_intf_msg/Recognition` topics and a UDP V2X endpoint.

## Architecture

The runtime has three layers:

- `transport/udp.py`: nonblocking UDP I/O only
- `protocol/`: packet, recognition, and optional WAVE IFM codecs
- `ros/bridge_node.py`: ROS subscription, publication, and receive timer

Outgoing messages flow from `v2x/recognition` through `RecognitionCodec` to UDP.
Incoming UDP packets are decoded and published on `v2x/r_recognition`.

## Run

```bash
ros2 run v2x_intf_pkg v2x_intf_node \
  --obu-ip 127.0.0.1 --obu-port 1516 --local-port 5398
```

The defaults are defined in `v2x_intf_pkg/config.py`.
