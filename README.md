![total lines](https://img.shields.io/tokei/lines/github/amaarquadri/fydp)

# Autonomous Rail Inspection System's Fourth Year Design Project
An all-inclusive beach resort. Unscented and Extended Kalman Filters,
Controllers, and control systems for a 25 state autonomous plane.

ROS Topics:
- raw_image
- rectified_image
- rail_detection
- optical_flow_reading
- pressure_sensor_reading
- imu_reading
- gps_reading
- aircraft_state
- reference_command
- state_command
- control_inputs

ROS Nodes:
- camera_node
- pressure_sensor_node
- imu_node
- gps_node
- defisheye_node
- rail_detection_node
- optical_flow_node
- state_estimation_node
- global_path_planner_node
- local_path_planner_node
- low_level_control_loop_node
- propeller_node
- aileron_node
- elevator_node

Reference Frames:
- Gazebo reference frame (absolute): the x-axis points north, the y-axis points west, 
the z-axis points upwards, and the origin is at the startup location of the robot at an altitude of zero
- Flat earth (NED) reference frame (absolute): the x-axis points north, the y-axis points east, 
the z-axis points down, and the origin is at the startup location of the robot at an altitude of zero
- Robot reference frame (moving): the x-axis points forwards, the y-axis points to the right (starboard), 
the z-axis points downwards, and the origin is at the center of mass of the aircraft
