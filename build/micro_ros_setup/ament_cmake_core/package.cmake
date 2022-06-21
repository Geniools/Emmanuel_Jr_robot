set(_AMENT_PACKAGE_NAME "micro_ros_setup")
set(micro_ros_setup_VERSION "2.1.0")
set(micro_ros_setup_MAINTAINER "Ingo Lütkebohle <ingo.luetkebohle@de.bosch.com>")
set(micro_ros_setup_BUILD_DEPENDS "flex" "bison" "libncurses-dev" "usbutils" "curl" "python3-yaml" "clang-tidy")
set(micro_ros_setup_BUILDTOOL_DEPENDS "ament_cmake" "asio" "tinyxml2")
set(micro_ros_setup_BUILD_EXPORT_DEPENDS "curl" "python3-yaml" "clang-tidy")
set(micro_ros_setup_BUILDTOOL_EXPORT_DEPENDS )
set(micro_ros_setup_EXEC_DEPENDS "python3-vcstool" "curl" "python3-yaml" "clang-tidy")
set(micro_ros_setup_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(micro_ros_setup_GROUP_DEPENDS )
set(micro_ros_setup_MEMBER_OF_GROUPS )
set(micro_ros_setup_DEPRECATED "")
set(micro_ros_setup_EXPORT_TAGS)
list(APPEND micro_ros_setup_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
