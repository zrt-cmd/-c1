#!/usr/bin/env python3
"""baseline14: mecanum lateral motion enabled in DWB + velocity smoother."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline11.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline14.yaml"
d = yaml.safe_load(open(src))

fp = d["controller_server"]["ros__parameters"]["FollowPath"]
fp["max_vel_y"] = 0.20
fp["min_vel_y"] = -0.20
fp["vy_samples"] = 10
fp["acc_lim_y"] = 1.0
fp["decel_lim_y"] = -1.0
fp["max_speed_xy"] = 0.35

vs = d["velocity_smoother"]["ros__parameters"]
vs["max_velocity"] = [0.4, 0.25, 3.0]
vs["min_velocity"] = [-0.4, -0.25, -3.0]

yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
print("max_vel_y", fp["max_vel_y"], "vy_samples", fp["vy_samples"])
print("smoother max", vs["max_velocity"])
