#!/usr/bin/env python3
"""Create a sim-correct Nav2 params template from the repo nav2_params.yaml."""
import os
import yaml

HOME = os.path.expanduser("~")
src = os.path.join(
    HOME, "ros2_ws/src/xflz_eai_car1/navigation/car1_navigation/config/nav2_params.yaml")
dst = os.path.join(HOME, "tune/nav2_sim_template.yaml")
install = os.path.join(HOME, "ros2_ws/install/car1_navigation/share/car1_navigation")
map_yaml = os.path.join(
    HOME, "ros2_ws/src/xflz_eai_car1/slam/car1_slam/maps/math_room.yaml")

with open(src, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

# 1. use_sim_time everywhere
for node, cfg in data.items():
    if isinstance(cfg, dict):
        rp = cfg.get("ros__parameters")
        if isinstance(rp, dict):
            rp["use_sim_time"] = True
# nested costmaps
for key in ("local_costmap", "global_costmap"):
    rp = data.get(key, {}).get(key, {}).get("ros__parameters")
    if isinstance(rp, dict):
        rp["use_sim_time"] = True

# 2. bt_navigator sim settings
bt = data["bt_navigator"]["ros__parameters"]
bt["odom_topic"] = "/odom"
bt["default_nav_to_pose_bt_xml"] = os.path.join(
    install, "config/behavior_trees/navigate_to_pose_without_backup.xml")
bt["default_nav_through_poses_bt_xml"] = os.path.join(
    install, "config/behavior_trees/navigate_through_poses_without_backup.xml")
# Jazzy loads built-in BT plugins automatically; the old explicit list causes
# duplicate node registration ("ID [ComputePathToPose] already registered").
bt.pop("plugin_lib_names", None)
bt["error_code_names"] = ["compute_path_error_code", "follow_path_error_code"]

# 3. AMCL: start exactly at G
amcl = data["amcl"]["ros__parameters"]
amcl["initial_pose"] = {"x": 0.15, "y": 1.70, "z": 0.0, "yaw": 0.0}
amcl["initial_cov_xx"] = 0.0025
amcl["initial_cov_yy"] = 0.0025
amcl["initial_cov_aa"] = 0.01
amcl["set_initial_pose"] = True
amcl["scan_topic"] = "/scan"

# 4. DWB: check the real footprint, not just the center point
fp = data["controller_server"]["ros__parameters"]["FollowPath"]
if "ObstacleFootprint" not in fp["critics"]:
    fp["critics"].insert(fp["critics"].index("BaseObstacle") + 1,
                         "ObstacleFootprint")
fp["ObstacleFootprint.scale"] = 0.01

# 4b. prefer corridor centerlines over shortest wall-hugging paths
data["planner_server"]["ros__parameters"]["GridBased"]["w_traversal_cost"] = 8.0

# 5. map + odom topics
data["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml
data["velocity_smoother"]["ros__parameters"]["odom_topic"] = "/odom"
data["controller_server"]["ros__parameters"]["odom_topic"] = "/odom"

os.makedirs(os.path.dirname(dst), exist_ok=True)
with open(dst, "w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

print("saved", dst)
print("BT xml:", bt["default_nav_to_pose_bt_xml"])
print("map:", map_yaml)
