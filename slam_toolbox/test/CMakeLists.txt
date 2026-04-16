catkin_add_gtest(${PROJECT_NAME}_unittest
    session_label_test.cpp
)

target_link_libraries(${PROJECT_NAME}_unittest
    toolbox_common
    yaml-cpp
    ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_remap_bbox_test
    remap_bbox_test.cpp
)

target_link_libraries(${PROJECT_NAME}_remap_bbox_test
    toolbox_common
    yaml-cpp
    ${catkin_LIBRARIES}
)

find_package(rostest REQUIRED)

add_rostest_gtest(${PROJECT_NAME}_ceres_fixed_test
    ceres_solver_fixed_test.test
    ceres_solver_fixed_test.cpp
)

target_link_libraries(${PROJECT_NAME}_ceres_fixed_test
    ceres_solver_plugin
    toolbox_common
    yaml-cpp
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
)