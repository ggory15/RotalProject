cmake_minimum_required(VERSION 3.5)
project(kjj_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  configs
  maps
  worlds
  robots
  components
  models
  DESTINATION share/${PROJECT_NAME})

ament_package()
