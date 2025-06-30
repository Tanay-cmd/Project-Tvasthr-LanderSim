cmake_minimum_required(VERSION 3.10)
project(moon_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/imu_reader.py
  src/alti_reader.py
  src/contact_sensors.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
