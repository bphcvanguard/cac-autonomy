cmake_minimum_required(VERSION 3.8)
project(cac_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(cac_sim_gen)

install(DIRECTORY config layouts DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_arena DESTINATION lib/${PROJECT_NAME})

# Generate the default worlds at build time and install them to share/cac_sim/worlds/:
# schematic, empty, seed_0001 .. seed_0005. Re-runs when the config, layouts, python or script change.
# All URIs in the SDF are relative, so the installed directories are relocatable.
file(GLOB_RECURSE _gen_deps CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/config/*.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/layouts/*.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/cac_sim_gen/*.py)
set(_worlds_dir ${CMAKE_CURRENT_BINARY_DIR}/worlds)
set(_worlds_stamp ${CMAKE_CURRENT_BINARY_DIR}/worlds.stamp)
set(_gen ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
         python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_arena)
add_custom_command(
  OUTPUT ${_worlds_stamp}
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${_worlds_dir}
  COMMAND ${_gen} --layout schematic --out ${_worlds_dir}
  COMMAND ${_gen} --layout empty --out ${_worlds_dir}
  COMMAND ${_gen} --layout random --seed 1 --count 5 --out ${_worlds_dir}
  COMMAND ${CMAKE_COMMAND} -E touch ${_worlds_stamp}
  DEPENDS ${_gen_deps} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_arena
  COMMENT "cac_sim: generating default arena worlds"
  VERBATIM)
add_custom_target(cac_sim_worlds ALL DEPENDS ${_worlds_stamp})
install(DIRECTORY ${_worlds_dir}/ DESTINATION share/${PROJECT_NAME}/worlds)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(cac_sim_tests test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600)
endif()

ament_package()
