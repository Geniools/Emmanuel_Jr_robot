# CMake generated Testfile for 
# Source directory: /home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_rotation_shim_controller/test
# Build directory: /home/snake/Emmanuel_Jr_robot/build/nav2_rotation_shim_controller/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_shim_controller "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/snake/Emmanuel_Jr_robot/build/nav2_rotation_shim_controller/test_results/nav2_rotation_shim_controller/test_shim_controller.gtest.xml" "--package-name" "nav2_rotation_shim_controller" "--output-file" "/home/snake/Emmanuel_Jr_robot/build/nav2_rotation_shim_controller/ament_cmake_gtest/test_shim_controller.txt" "--command" "/home/snake/Emmanuel_Jr_robot/build/nav2_rotation_shim_controller/test/test_shim_controller" "--gtest_output=xml:/home/snake/Emmanuel_Jr_robot/build/nav2_rotation_shim_controller/test_results/nav2_rotation_shim_controller/test_shim_controller.gtest.xml")
set_tests_properties(test_shim_controller PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/snake/Emmanuel_Jr_robot/build/nav2_rotation_shim_controller/test/test_shim_controller" TIMEOUT "60" WORKING_DIRECTORY "/home/snake/Emmanuel_Jr_robot/build/nav2_rotation_shim_controller/test" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/galactic/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_rotation_shim_controller/test/CMakeLists.txt;4;ament_add_gtest;/home/snake/Emmanuel_Jr_robot/src/navigation2/nav2_rotation_shim_controller/test/CMakeLists.txt;0;")
subdirs("../gtest")
