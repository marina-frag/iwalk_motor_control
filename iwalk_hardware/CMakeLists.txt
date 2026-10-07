cmake_minimum_required(VERSION 3.8)
project(iwalk_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(iwalk_vesc_protocol STATIC src/vesc_protocol.cpp)
target_compile_features(iwalk_vesc_protocol PUBLIC cxx_std_17)
target_include_directories(iwalk_vesc_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(iwalk_vesc_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED src/vesc_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} iwalk_vesc_protocol)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(vesc_waveshare_bridge src/waveshare_bridge.cpp)
target_compile_features(vesc_waveshare_bridge PUBLIC cxx_std_17)
target_link_libraries(vesc_waveshare_bridge PRIVATE iwalk_vesc_protocol)

pluginlib_export_plugin_description_file(
  hardware_interface iwalk_hardware.xml
)

install(TARGETS
  ${PROJECT_NAME}
  iwalk_vesc_protocol
  vesc_waveshare_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_vesc_protocol test/test_vesc_protocol.cpp)
  target_link_libraries(test_vesc_protocol iwalk_vesc_protocol)

  ament_add_gtest(test_vesc_system test/test_vesc_system.cpp)
  target_link_libraries(test_vesc_system ${PROJECT_NAME} iwalk_vesc_protocol)
  ament_target_dependencies(test_vesc_system
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} iwalk_vesc_protocol)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

ament_package()
