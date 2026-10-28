cmake_minimum_required(VERSION 3.8)
project(cac_description)
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(cac_description_tests test)
endif()

ament_package()
