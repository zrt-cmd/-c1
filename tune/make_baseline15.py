#!/usr/bin/env python3
"""baseline15: add the path smoother so corners become arcs."""
import yaml

src = "/home/zrtzrt/tune/nav2_baseline11.yaml"
dst = "/home/zrtzrt/tune/nav2_baseline15.yaml"
d = yaml.safe_load(open(src))

d["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = (
    "/home/zrtzrt/ros2_ws/install/car1_navigation/share/car1_navigation/"
    "config/behavior_trees/navigate_to_pose_smooth.xml")

names = d["lifecycle_manager_navigation"]["ros__parameters"]["node_names"]
if "smoother_server" not in names:
    names.insert(names.index("planner_server") + 1, "smoother_server")

d["smoother_server"] = {
    "ros__parameters": {
        "use_sim_time": True,
        "smoother_plugins": ["SimpleSmoother"],
        "SimpleSmoother": {
            "plugin": "nav2_smoother::SimpleSmoother",
            "tolerance": 1.0e-10,
            "max_iterations": 1000,
            "do_refinement": True,
        },
    }
}

yaml.safe_dump(d, open(dst, "w"), sort_keys=False, allow_unicode=True)
print("wrote", dst)
print("lifecycle nodes:", names)
