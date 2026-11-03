#!/usr/bin/env python3
"""baseline17: Regulated Pure Pursuit local controller (no trajectory sampling)."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline16.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline17.yaml"
d = yaml.safe_load(open(src))

fp = d["controller_server"]["ros__parameters"]["FollowPath"] = {
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "desired_linear_vel": 0.15,
    "lookahead_dist": 0.35,
    "min_lookahead_dist": 0.15,
    "max_lookahead_dist": 0.55,
    "lookahead_time": 1.5,
    "use_velocity_scaled_lookahead_dist": True,
    "min_approach_linear_velocity": 0.03,
    "approach_velocity_scaling_dist": 0.3,
    "use_collision_detection": True,
    "max_allowed_time_to_collision_up_to_carrot": 1.0,
    "use_regulated_linear_velocity_scaling": True,
    "use_cost_regulated_linear_velocity_scaling": True,
    "regulated_linear_scaling_min_radius": 0.9,
    "regulated_linear_scaling_min_speed": 0.1,
    "use_rotate_to_heading": True,
    "rotate_to_heading_min_angle": 0.6,
    "rotate_to_heading_angular_vel": 0.8,
    "max_angular_accel": 1.5,
    "allow_reversing": False,
    "transform_tolerance": 0.2,
}

yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
print("controller:", fp["plugin"])
