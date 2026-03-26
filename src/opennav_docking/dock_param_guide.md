# Main dock controller param
```yaml
Rush:
    max_linear_vel: 0.3
    # The max linear speed dock can obtain. Ideally, dock should achieve max vel
    min_linear_vel: 0.01
    # The min linear speed. Useful when really close to goal.
        # The above two can be tested with `ros topic pub`
    linear_kp_accel_vel: 0.6
    linear_ki_accel_vel: 0.2
    # In `controller.cpp`, `Acceleration()`:
    # `vel += linear_kp_accel_vel_ * fabs(vel - max_linear_vel_) + linear_ki_accel_vel_ * vel_error_sum_;`
    # Remark: These kp, ki are regarding v
    linear_kp_decel_dis: 0.15
    # In `controller.cpp`, `Deceleration()`:
    # `double raw_vel = linear_kp_decel_dis_ * decel_dist_error;`
    # note that this is raw vel
    max_speed_diff: 0.6
    # In `controller.cpp`, `Deceleration()`:
    # `double max_delta = max_speed_diff_ * dt; double speed_diff = raw_vel - previous_speed_;`
    # Deceleration cannot have `a` that is greater than `max_speed_diff`
    # This is to avoid the dilemma:
        # linear_kp_decel_dis too large -> when close goal, speed still too large
        # linear_kp_decle_dis too small -> when transition from CONSTANT to DECELERATION, there's a velocity jump
    # This feature is created by DIT 13th Chuang, T.H
    angular_kp: 0.2
    # kp of angular difference
    deceleration_distance: 0.16
    # distance between robot and goal to transition into DECELERATION
    # if too large, casue dock directly enter DECELERATION, then the speed will be very small
    reserved_distance: 0.0075
    # distance between robot and goal, so that v directly set to `min_linear_vel`
    external_rival_data_path: "/home/share/data/rival_params.yaml"
    stop_degree: 120.0
    rival_radius: 0.44
    # obstacle avoiding, but not planned to be used in 2026
```

# some other dock param
note: these are upward
- `docking_threshold`: threshold that dock regard as arrived
- `external_detection_timeout`: camera topic timeout
- `filter_coef`: set to 1.0. Camera should cover the filter.
- `camera_aruco_max`: the max distance that cam can see ArUco. This affect overall docking distance, Don't set too large
- `camera_aruco_min`: the min distance that cam can see ArUco.
  - These two will determine the `staging_dist`
  - So don't set max to too large.