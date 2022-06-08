# CMake generated Testfile for 
# Source directory: /home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/src/system_failure
# Build directory: /home/snake/Emmanuel_Jr_robot/build/nav2_system_tests/src/system_failure
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_failure_navigator "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/snake/Emmanuel_Jr_robot/build/nav2_system_tests/test_results/nav2_system_tests/test_failure_navigator.xml" "--package-name" "nav2_system_tests" "--generate-result-on-success" "--env" "TEST_DIR=/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/src/system_failure" "TEST_MAP=/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/maps/map_circular.yaml" "TEST_WORLD=/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/worlds/turtlebot3_ros2_demo.world" "GAZEBO_MODEL_PATH=/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/models" "BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml" "--command" "/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/src/system_failure/test_system_failure_launch.py")
set_tests_properties(test_failure_navigator PROPERTIES  TIMEOUT "180" WORKING_DIRECTORY "/home/snake/Emmanuel_Jr_robot/build/nav2_system_tests/src/system_failure" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/src/system_failure/CMakeLists.txt;1;ament_add_test;/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_system_tests/src/system_failure/CMakeLists.txt;0;")
