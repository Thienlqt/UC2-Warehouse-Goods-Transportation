# uc2_vision_msgs

The detection messages this project uses, in the layout of ROS 2
[vision_msgs](https://github.com/ros-perception/vision_msgs) 4.2 (branch `ros2`), built for
ROS 1 Noetic. Noetic's own `vision_msgs` differs: `ObjectHypothesisWithPose.id` is an int64
instead of a string `class_id`, `Detection2D` / `Detection3D` have no `id` for track ids, and
`BoundingBox2D.center` is a flat `x, y, theta`.

Unity's generated C# classes (`UnityProject/Assets/Scripts/RosMessages/Vision`) match these
definitions field for field. The `.msg` files are copied unchanged from vision_msgs 4.2 apart
from package-relative references. Apache-2.0, see `LICENSE`.
