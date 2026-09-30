Component name: ROBOTIS TurtleBot3

License Type: Apache 2.0

Copyright 2016 ROBOTIS Co.

```
Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the 
License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed 
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR 
CONDITIONS OF ANY KIND, either express or implied. See the License for the 
specific language governing permissions and limitations under the License.
```

---

Component name: FreeCam.cs

License Type: MIT

```
Copyright © 2019 Ashley Davis

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

---

Component name: vision_msgs (message definitions in catkin_ws/src/uc2_vision_msgs)

License Type: Apache 2.0

Copyright Open Source Robotics Foundation and vision_msgs contributors

```
Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the 
License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed 
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR 
CONDITIONS OF ANY KIND, either express or implied. See the License for the 
specific language governing permissions and limitations under the License.
```

---

Component name: Robotics-Nav2-SLAM-Example (the upstream project this repository is derived from)

License Type: Apache 2.0

Copyright 2021 Unity Technologies

Source: https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example. Its license is
[LICENSE.md](LICENSE.md). Files taken from it (Unity project, scene, scripts, TurtleBot3 assets,
ROS package) have been modified; see "Credits and license" in README.md.

---

Component name: ROS-TCP-Endpoint v0.6.0, ROS 1 release (catkin_ws/src/ROS-TCP-Endpoint)

License Type: Apache 2.0

Copyright 2020 Unity Technologies

Vendored with its license (catkin_ws/src/ROS-TCP-Endpoint/LICENSE) and copyright headers.
Modified: `default_server_endpoint.py` runs with `python3`; see its CHANGELOG.md.

---

Component name: Robotics-Warehouse (Unity package com.unity.robotics.warehouse, branch nav2-example)

License Type: Apache 2.0

Copyright Unity Technologies

Not stored in this repository: Unity's Package Manager downloads it from
https://github.com/Unity-Technologies/Robotics-Warehouse. The detector training frames are
rendered from its assets.

---

Component name: slam_toolbox parameters (catkin_ws/src/unity_slam_example/config/slam_toolbox.yaml)

License Type: LGPL 2.1

Copyright Samsung Research America and slam_toolbox contributors

The parameter values are those of slam_toolbox's `config/mapper_params_online_async.yaml`
(https://github.com/SteveMacenski/slam_toolbox, Noetic release).

---

Component name: Ultralytics YOLOv8 (catkin_ws/src/unity_slam_example/models/detector.onnx)

License Type: AGPL 3.0

Copyright Ultralytics

`detector.onnx` is YOLOv8n (pretrained on COCO) fine-tuned with the Ultralytics package by
`detection_training/train.sh`. Ultralytics distributes its models and software under AGPL-3.0
(https://github.com/ultralytics/ultralytics/blob/main/LICENSE), or under a separate enterprise
licence. The detector model is therefore not covered by this repository's Apache-2.0 licence:
it is distributed under AGPL-3.0, with the licence text in
catkin_ws/src/unity_slam_example/models/LICENSE. The inference code (`perception/yolo.py`) is this project's own and does not use Ultralytics.
