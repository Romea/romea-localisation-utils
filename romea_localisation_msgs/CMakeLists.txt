cmake_minimum_required(VERSION 3.5)
project(romea_localisation_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(romea_common_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/ObservationAngularSpeed.msg
    msg/ObservationAngularSpeedStamped.msg
    msg/ObservationAttitude.msg
    msg/ObservationAttitudeStamped.msg
    msg/ObservationCourse.msg
    msg/ObservationCourseStamped.msg
    msg/ObservationRange.msg
    msg/ObservationRangeStamped.msg
    msg/ObservationPosition2D.msg
    msg/ObservationPosition2DStamped.msg
    msg/ObservationPose2D.msg
    msg/ObservationPose2DStamped.msg
    msg/ObservationTwist2DStamped.msg
    msg/ObservationTwist2D.msg
#    srv/CheckLocalisationStatus.srv
    DEPENDENCIES std_msgs romea_common_msgs
    )

ament_export_dependencies(rosidl_default_runtime)
ament_package()


#cmake_minimum_required(VERSION 2.8.3)
#project(romea_localisation_msgs)

#set(${PROJECT_NAME}_CATKIN_DEPS std_msgs geometry_msgs message_generation)
#find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

#add_message_files(
#  DIRECTORY msg
#  FILES
#  AngularSpeed.msg
#  AngularSpeedStamped.msg
#  Attitude.msg
#  AttitudeStamped.msg
#  Course.msg
#  CourseStamped.msg
#  Range.msg
#  RangeStamped.msg
#  Position2D.msg
#  Pose2D.msg
#  Twist2D.msg
#  PoseAndTwist2D.msg
#  Position2DStamped.msg
#  Pose2DStamped.msg
#  Twist2DStamped.msg
#  PoseAndTwist2DStamped.msg
#  )

#add_service_files(
#  DIRECTORY srv
#  FILES
#  CheckLocalisationStatus.srv
#  )

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  )

#catkin_package(
#  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
#  )
