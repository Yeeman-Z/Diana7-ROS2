cmake_minimum_required(VERSION 3.8)
project(diana7_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/diana7_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    hardware_interface
    pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Vendor SDK integration
set(DIANA_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/DianaApi-x86_64-x64-linux/DianaApi)
set(DIANA_SDK_INCLUDE ${DIANA_SDK_DIR}/include)
set(DIANA_SDK_BIN ${DIANA_SDK_DIR}/bin)
set(DIANA_SDK_LIB ${DIANA_SDK_DIR}/lib)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${DIANA_SDK_INCLUDE}
)

# Link against vendor shared libraries (order can matter if there are deps)
set(DIANA_VENDOR_LIBS
    ${DIANA_SDK_BIN}/libDianaApi.so
    ${DIANA_SDK_BIN}/libVersionApi.so
    ${DIANA_SDK_BIN}/libToolSdk.so
    ${DIANA_SDK_BIN}/libBasicSdk.so
    ${DIANA_SDK_BIN}/libGenericAlgorithm.so
    ${DIANA_SDK_BIN}/libxml2.so
)

target_link_libraries(${PROJECT_NAME}
    ${DIANA_VENDOR_LIBS}
)

# Ensure runtime can find .so in install/lib without extra env vars
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../lib"
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(FILES plugin_description.xml
    DESTINATION share/${PROJECT_NAME}
)

# Install vendor headers and libraries into install space
install(DIRECTORY ${DIANA_SDK_INCLUDE}/
    DESTINATION include/vendor/DianaApi
)
install(DIRECTORY ${DIANA_SDK_BIN}/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so" PATTERN "*.py" EXCLUDE
)
install(DIRECTORY ${DIANA_SDK_LIB}/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so"
)

# Brake service node
add_executable(brake_service_node src/brake_service_node.cpp)
target_include_directories(brake_service_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${DIANA_SDK_INCLUDE}
)
ament_target_dependencies(brake_service_node rclcpp std_srvs)
target_link_libraries(brake_service_node ${DIANA_VENDOR_LIBS})
set_target_properties(brake_service_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../lib"
)

add_executable(speedl_line_test src/speedl_line_test.cpp)
target_include_directories(speedl_line_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${DIANA_SDK_INCLUDE}
)
target_link_libraries(speedl_line_test ${DIANA_VENDOR_LIBS})
set_target_properties(speedl_line_test PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../lib"
)

install(TARGETS brake_service_node speedl_line_test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
