cmake_minimum_required(VERSION 3.8)
project(vamp_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(core_types REQUIRED)
find_package(motion_planner REQUIRED)
find_package(Eigen3 REQUIRED)

# VAMP is an unmodified, pinned upstream built by docker/Dockerfile (target vamp-build) and installed
# to /opt/vamp. Without it this package builds nothing, so the rest of the workspace is unaffected.
# Build flags must match how VAMP was built (AVX2) because its vector types are compiled into our
# translation units; override with -DVAMP_PLANNER_ARCH=... for a different baseline.
# Upstream's installed vampConfig.cmake is unusable as shipped (it calls find_dependency without the
# macro, exports vamp::vamp_cpp but checks for vamp::vamp, and points the include path one level too
# deep). Upstream stays unmodified, so we describe the installed headers and the two static libs
# ourselves.
set(VAMP_ROOT "/opt/vamp" CACHE PATH "VAMP install prefix")
if(NOT EXISTS "${VAMP_ROOT}/include/vamp/vector.hh")
  message(WARNING "VAMP not found at ${VAMP_ROOT}: skipping vamp_planner")
  ament_package()
  return()
endif()
set(VAMP_LIBS "")
foreach(lib simdxorshift cdd)
  find_library(VAMP_${lib}_LIB ${lib} PATHS ${VAMP_ROOT}/lib NO_DEFAULT_PATH)
  if(VAMP_${lib}_LIB)
    list(APPEND VAMP_LIBS ${VAMP_${lib}_LIB})
  endif()
endforeach()

set(VAMP_PLANNER_ARCH "-march=x86-64-v3;-mavx2" CACHE STRING "SIMD flags matching the VAMP build")

# Robot models are compiled-in VAMP headers, one per robot. The bundled ones need no generation;
# ours come from FOAM + Cricket (docker/tools/generate_vamp_robot.sh). Point
# VAMP_PLANNER_ROBOT_DIR at a directory of generated <Name>.hh files and each one is compiled in and
# registered under its own `name` (no source edits). Headers are large: keep them out of git.
set(VAMP_PLANNER_ROBOT_DIR "" CACHE PATH "Directory of Cricket-generated VAMP robot headers")
set(GEN_INC ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GEN_INC})
set(INCLUDES_TEXT "")
set(TABLE_TEXT "")
if(VAMP_PLANNER_ROBOT_DIR)
  file(GLOB GENERATED_HEADERS ${VAMP_PLANNER_ROBOT_DIR}/*.hh)
  foreach(h ${GENERATED_HEADERS})
    file(STRINGS ${h} STRUCT_LINE REGEX "^struct [A-Za-z_][A-Za-z0-9_]*$" LIMIT_COUNT 1)
    if(STRUCT_LINE)
      string(REGEX REPLACE "^struct " "" STRUCT_NAME "${STRUCT_LINE}")
      string(APPEND INCLUDES_TEXT "#include \"${h}\"\n")
      string(APPEND TABLE_TEXT "  {vamp::robots::${STRUCT_NAME}::name, &make<vamp::robots::${STRUCT_NAME}>},\n")
      message(STATUS "vamp_planner: generated robot ${STRUCT_NAME} (${h})")
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${h})
    endif()
  endforeach()
endif()
file(WRITE ${GEN_INC}/robots_includes.inc "${INCLUDES_TEXT}")
file(WRITE ${GEN_INC}/robots_table.inc "${TABLE_TEXT}")
add_library(vamp_planner
  src/vamp_planner.cpp
  src/robots.cpp
)
target_include_directories(vamp_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(vamp_planner PRIVATE ${VAMP_PLANNER_ARCH})
# Eigen types must not cross this library's boundary (ABI differs with the vector flags above).
target_link_libraries(vamp_planner PUBLIC motion_planner::motion_planner_core core_types::core_types
  PRIVATE Eigen3::Eigen ${VAMP_LIBS})
target_include_directories(vamp_planner SYSTEM PRIVATE ${VAMP_ROOT}/include)
target_include_directories(vamp_planner PRIVATE ${GEN_INC})
set_target_properties(vamp_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS vamp_planner EXPORT export_vamp_planner
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(vamp_planner_test test/vamp_planner_test.cpp)
  target_link_libraries(vamp_planner_test vamp_planner)
endif()

ament_export_targets(export_vamp_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(core_types motion_planner)
ament_package()
