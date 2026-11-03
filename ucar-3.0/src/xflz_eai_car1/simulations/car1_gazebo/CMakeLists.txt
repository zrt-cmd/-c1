cmake_minimum_required(VERSION 3.8)

project(car1_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch worlds config urdf models
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本为可执行文件
install(PROGRAMS
  scripts/grasp_demo.py
  scripts/manual_control.py
  scripts/grasp_demo_manual_ref.py
  scripts/object_detector.py
  scripts/grasp_demo_vision.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
