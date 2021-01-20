# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;tf2;tf2_ros;nodelet;mav_msgs;mav_planning_msgs;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldla2_path_planner_ros".split(';') if "-ldla2_path_planner_ros" != "" else []
PROJECT_NAME = "dla2_path_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.0.0"
