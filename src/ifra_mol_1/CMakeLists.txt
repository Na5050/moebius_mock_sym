cmake_minimum_required(VERSION 3.20)
project(ifra_mol_1)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
#find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. ГЕНЕРАЦИЯ СЕРВИСОВ
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AutopilotTime.srv"
  "srv/AutopilotRelative.srv"
  "msg/AutopilotTelemetry.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs std_srvs
)

ament_package()
