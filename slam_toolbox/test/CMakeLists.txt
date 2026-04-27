catkin_add_gtest(${PROJECT_NAME}_unittest
    session_state_test.cpp
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

catkin_add_gtest(${PROJECT_NAME}_polygon_fill_test
    polygon_fill_test.cpp
)

target_link_libraries(${PROJECT_NAME}_polygon_fill_test
    ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_polygon_units_test
    polygon_units_test.cpp
)

target_link_libraries(${PROJECT_NAME}_polygon_units_test
    toolbox_common
    ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_labels_serialization_test
    labels_serialization_test.cpp
)

target_link_libraries(${PROJECT_NAME}_labels_serialization_test
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