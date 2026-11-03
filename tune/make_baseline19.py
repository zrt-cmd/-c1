#!/usr/bin/env python3
"""baseline19: RPP without its (false-positive) collision detection."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline18.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline19.yaml"
d = yaml.safe_load(open(src))
d["controller_server"]["ros__parameters"]["FollowPath"]["use_collision_detection"] = False
yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
