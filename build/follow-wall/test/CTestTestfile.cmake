# CMake generated Testfile for 
# Source directory: /home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/test
# Build directory: /home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/build/follow-wall/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(node_test "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/build/follow-wall/test_results/follow-wall/node_test.gtest.xml" "--package-name" "follow-wall" "--output-file" "/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/build/follow-wall/ament_cmake_gtest/node_test.txt" "--command" "/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/build/follow-wall/test/node_test" "--gtest_output=xml:/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/build/follow-wall/test_results/follow-wall/node_test.gtest.xml")
set_tests_properties(node_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/build/follow-wall/test/node_test" TIMEOUT "60" WORKING_DIRECTORY "/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/build/follow-wall/test" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/test/CMakeLists.txt;1;ament_add_gtest;/home/alumnos/acmp/colcon_ws/src/follow-wall-roboros/test/CMakeLists.txt;0;")
subdirs("../gtest")
