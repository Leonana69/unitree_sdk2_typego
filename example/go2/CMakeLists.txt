# add_executable(go2_trajectory_follow go2_trajectory_follow.cpp)
# target_link_libraries(go2_trajectory_follow unitree_sdk2)

# add_executable(go2_sport_client go2_sport_client.cpp)
# target_link_libraries(go2_sport_client unitree_sdk2)

# add_executable(go2_low_level go2_low_level.cpp)
# target_link_libraries(go2_low_level unitree_sdk2)

# add_executable(go2_stand_example go2_stand_example.cpp)
# target_link_libraries(go2_stand_example unitree_sdk2)

# add_executable(go2_dashboard go2_dashboard.cpp)
# target_link_libraries(go2_dashboard unitree_sdk2)

# add_executable(go2_robot_state_client go2_robot_state_client.cpp)
# target_link_libraries(go2_robot_state_client unitree_sdk2)

# add_executable(go2_video_client go2_video_client.cpp)
# target_link_libraries(go2_video_client unitree_sdk2)

# add_executable(go2_vui_client go2_vui_client.cpp)
# target_link_libraries(go2_vui_client unitree_sdk2)
add_executable(typego_control_interface typego_control_interface.cpp)
target_link_libraries(typego_control_interface unitree_sdk2)

add_executable(typego_high_state_service typego_high_state_service.cpp)
target_link_libraries(typego_high_state_service unitree_sdk2)

add_executable(typego_audio_interface typego_audio_interface.cpp)
target_link_libraries(typego_audio_interface unitree_sdk2 asound)

add_executable(typego_livox_service typego_livox_service.cpp)

# Include Livox SDK headers
target_include_directories(typego_livox_service PRIVATE /usr/local/include)

# Link against Livox SDK shared library, static library (for LivoxLidarUpgrader), and the Unitree SDK
target_link_libraries(typego_livox_service
    unitree_sdk2
    /usr/local/lib/liblivox_lidar_sdk_shared.so
    /usr/local/lib/liblivox_lidar_sdk_static.a
)

# Install example executables
install(TARGETS
    typego_control_interface
    typego_high_state_service
    typego_livox_service
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)