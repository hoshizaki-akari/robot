cmake_minimum_required(VERSION 3.8)
project(aubo_i5_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/aubo_i5_description
)

install(FILES LICENSE README.md PROVENANCE.md stage_report.txt
  DESTINATION share/aubo_i5_description
)

ament_package()
