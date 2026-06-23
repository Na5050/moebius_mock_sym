cmake_minimum_required(VERSION 3.12) # Исправлено предупреждение (заменили 3.8 на 3.12)
project(robot_autopilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# 2. УСТАНОВКА PYTHON СКРИПТА
# ИСПРАВЛЕНО: Указано новое правильное имя переименованного файла скрипта!
install(PROGRAMS
  scripts/autopilot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

