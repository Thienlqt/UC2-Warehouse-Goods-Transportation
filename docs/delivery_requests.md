# Delivery requests and sequential navigation

The `delivery_controller` ROS node starts with the regular launch file. Each request has three
poses in the **ROS `map` frame**: pickup (STORAGE), dropoff (STATION), then HOME. It sends one
goal at a time to the `move_base` action server and waits for its result before advancing.
Requests are handled in arrival order. If any leg fails, is canceled, or times out, the
remaining legs are skipped and the next queued request can run.

## Use from Unity

Open `SimpleWarehouseScene` and press Play after the ROS side is running. The **Delivery
requests (map frame)** panel appears at the upper right. Enter the pickup, dropoff and home
`x`, `y` coordinates in metres and `yaw` in radians. Choose reachable points on the mapped
free floor in RViz; the initial `(0, 0)` fields are placeholders, not verified warehouse
coordinates. Press **Queue delivery** more than once to see the FIFO queue. The panel displays the latest status
and the active and pending IDs. Enter one of those IDs in the cancel field and press
**Cancel ID** to remove it or stop its active goal.

Other Unity scripts can call `DeliveryRequestPanel.EnqueueDelivery(pickupX, pickupY, pickupYaw,
dropoffX, dropoffY, dropoffYaw, homeX, homeY, homeYaw)`. It returns the generated request ID. Call
`CancelDelivery(id)` to cancel an active or queued request.

## Use from a ROS shell

In the container, open a shell with:

```bash
docker exec -it -u ubuntu -e HOME=/home/ubuntu unity-nav2 bash
```

Request a delivery (substitute reachable map coordinates):

```bash
rostopic pub -1 /delivery/request std_msgs/String '{data: "{\"id\":\"order-001\",\"pickup\":{\"x\":1.0,\"y\":2.0,\"yaw\":0.0},\"dropoff\":{\"x\":3.0,\"y\":2.0,\"yaw\":1.57},\"home\":{\"x\":0.0,\"y\":0.0,\"yaw\":0.0}}"}'
rostopic echo /delivery/status
rostopic echo /delivery/queue
```

`/delivery/status` reports `queued`, `navigating` (with throttled robot-position feedback),
`pickup_complete`, `dropoff_complete`, `completed`, `failed`, `canceling`, `canceled`,
`rejected`, or `not_found` with the request ID and leg. `/delivery/queue` is a latched JSON
snapshot of the active delivery and pending IDs.

To check the ROS topic and action wiring without Unity, build the catkin workspace, source
`devel/setup.bash`, then run `python3 src/unity_slam_example/test/check_delivery_action.py`
from the workspace root. This starts a temporary ROS master and simulated `move_base` action
server and checks that two requests dispatch all six goals in FIFO order.

Cancel an ID, or publish `*` to cancel the active delivery and empty the queue:

```bash
rostopic pub -1 /delivery/cancel std_msgs/String '{data: "order-001"}'
```

The controller waits for Unity's `/clock` and the `move_base` action server before starting a
goal. The default limit is 50 pending deliveries, and each leg has a 300-second wall-clock
timeout. Both can be changed in `launch/unity_slam_example.launch`. Manual RViz navigation
goals and assisted keyboard driving also use `move_base`; do not send them during an active
delivery because they can preempt its goal.
