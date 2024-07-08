# task_2
## The Custom Planner package provides a simple straight-line path planner plugin for integration with Navigation2 in ROS 2. This planner generates a path directly between the start and goal poses specified, moving in a straight line.
## custom_planner/
## ├── include/
## │   └── nav2_straightline_planner/
## │       └── straight_line_planner.hpp
## ├── src/
## │   └── straight_line_planner.cpp
## ├── launch/
## │   └── customm_planner_launch.py
## ├── CMakeLists.txt
## ├── package.xml
## └── global_planner_plugin.xml

## install Navigation2 stack: "git clone git clone https://github.com/ros-planning/navigation2.git"

# once the mapping is finished, and the map is saved.
## RUN TURTLEBOT : ### "ros2 launch task_1 house.launch.py"
## EXECUTE NAVIGATION : ### "ros2 launch custom_planner custom_planner_launch.py"

# BE SURE TO MODIFY THE NAV2_PARAMS.YAML FILE IN nav2_bringup:
  planner_server:
  ros__parameters:
    plugins: ["GridBased"]
    use_sim_time: True
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner" # For Foxy and later. In Iron and older versions, "/" was used instead of "::"
      tolerance: 2.0
      use_astar: false
      allow_unknown: true
## with

planner_server:
  ros__parameters:
    plugins: ["GridBased"]
    use_sim_time: True
    GridBased:
      plugin: "custom_planner::StraightLine"
      interpolation_resolution: 0.1

### /Reference/Authoe: shivang_patel
