catkin_add_gtest(${PROJECT_NAME}_unittest
    session_label_test.cpp
)

target_link_libraries(${PROJECT_NAME}_unittest
    toolbox_common
    yaml-cpp
    ${catkin_LIBRARIES}
)