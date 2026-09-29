cmake_minimum_required(VERSION 3.0.2)
project(uc2_vision_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(FILES
  Point2D.msg
  Pose2D.msg
  BoundingBox2D.msg
  BoundingBox3D.msg
  ObjectHypothesis.msg
  ObjectHypothesisWithPose.msg
  Detection2D.msg
  Detection2DArray.msg
  Detection3D.msg
  Detection3DArray.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
