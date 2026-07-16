# Rectangle vision ROS 2 node

This ROS 2 Jazzy node reads `/dev/video0`, detects the thick black rectangular
frame, draws a green centerline over the black border, and draws a red point at
the intersection of the quadrilateral diagonals. Detection and JPEG encoding run
on the Raspberry Pi; Foxglove only displays the published result.

The detector combines direct border measurement with optical-flow tracking and
short-term motion prediction, so it tolerates moderate motion blur, partial
occlusion, perspective changes, and slight bending of the printed rectangle.

## Build and run on the Raspberry Pi

```bash
cd ~/rectangle_visual
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rectangle_vision rectangle_vision.launch.py
```

The launch file starts Foxglove bridge on port 8765 by default. If a bridge is
already listening on that port, it reuses the existing bridge instead of
starting a duplicate. Pass `use_foxglove:=false` to disable this behavior.

Connect Foxglove to `ws://192.168.3.126:8765`, add an Image panel, and select
`/rectangle/image/compressed`. The unmodified camera stream is available as
`/camera/image_raw/compressed`.

Other topics:

- `/rectangle/detected` (`std_msgs/Bool`)
- `/rectangle/center` (`geometry_msgs/PointStamped`, pixel coordinates)
- `/rectangle/polygon` (`geometry_msgs/PolygonStamped`, pixel coordinates)

The camera user must belong to the `video` group. Log out and back in after
running `sudo usermod -aG video heximiao`.

For Raspberry Pi performance, detection runs at half resolution and full contour
detection runs every second frame; optical flow handles the frames in between.
The annotated stream remains 640x480. The raw JPEG stream is disabled by default
to avoid a second JPEG encode; enable it with `publish_raw:=true` if needed.
