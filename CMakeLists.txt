cmake_minimum_required(VERSION 3.16)
project(hlk)

set(PROJECT_NAME hlk)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_CURRENT_SOURCE_DIR}/ext/libQEx/cmake)

# OLD CMAKE FOR LIBIGL
# LibIGL options - may neet to use option instead of set
# option(LIBIGL_WITH_PNG "Use PNG" ON)
# option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
# option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
# option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use imGUI" ON)
# option(LIBIGL_WITH_COMISO "Use CoMiso" ON)
# option(LIBIGL_WITH_CGAL "Use CGAL" ON)

# # Note - many LIBIGL functions do not work as a static library
# # (for example, igl::copyleft::cgal::wire_mesh), and you have less
# # lee-way when creating matrices (safest to use MatrixXd and MatrixXi
# # rather than explicitly declaring dimensions). Comment this out
# # in case of such problems
# option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" ON)

# # Hack for libigl linking: https://github.com/libigl/libigl/issues/1229
# set(MSVC_RUNTIME "dynamic")

# Libigl
include(libigl)

# Enable the target igl::glfw
igl_include(glfw)
# Other modules you could enable
#igl_include(embree)
igl_include(imgui)
igl_include(opengl)
igl_include(stb)
#igl_include(predicates)
#igl_include(xml)
igl_include(copyleft cgal)
# igl_include(copyleft comiso)
igl_include(copyleft core)
#igl_include(copyleft cork)
#igl_include(copyleft tetgen)
#igl_include(restricted matlab)
#igl_include(restricted mosek)
#igl_include(restricted triangle)

# find_package(LIBIGL REQUIRED)
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
endif()
find_package(Z3 REQUIRED CONFIG)

add_subdirectory(ext/libQEx EXCLUDE_FROM_ALL)
add_subdirectory(ext/CoMISo EXCLUDE_FROM_ALL)
add_subdirectory(compat/libigl-comiso)
add_subdirectory(compat/libigl-png)
target_compile_definitions(QEx PRIVATE _USE_MATH_DEFINES)

# Just grab all C++ files
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB DEBUG_FILES ${PROJECT_SOURCE_DIR}/src/*.natvis)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES} ${DEBUG_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${Z3_CXX_INCLUDE_DIRS} ${QEX_INCLUDE_DIRS} ext/Directional/include)

# Link igl (and the glfw module) to your project
target_link_libraries(${PROJECT_NAME} PUBLIC 
  igl::glfw
  ## Other modules you could link to
  # igl::embree
  igl::imgui
  igl::opengl
  igl::stb
  hlk_libigl_png_compat
  # igl::predicates
  # igl::xml
  igl_copyleft::cgal
  # igl_copyleft::comiso
  hlk_libigl_comiso_compat
  igl_copyleft::core
  # igl_copyleft::cork
  # igl_copyleft::tetgen
  # igl_restricted::matlab
  # igl_restricted::mosek
  # igl_restricted::triangle
	Eigen3::Eigen
  ${Z3_LIBRARIES}
	QEx)

target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

# Copy Runtime Resources to Binary Directory
file(GLOB RESOURCE_FILES ${PROJECT_SOURCE_DIR}/resources/*)
foreach(RESOURCE_FILE ${RESOURCE_FILES})
  file(COPY ${RESOURCE_FILE} DESTINATION ${PROJECT_BINARY_DIR})
endforeach(RESOURCE_FILE)

# Copy shared libraries to binary directory on Windows
if("${CMAKE_SYSTEM_NAME}" MATCHES "[Ww]indows")
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
  igl_copy_imported_dll(CoMISo stitchgraph)
  foreach (z3_lib ${Z3_LIBRARIES})
    message(STATUS "Adding copy rule for ${z3_lib}")
    add_custom_command(TARGET ${PROJECT_NAME}
      POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${z3_lib}>
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
  endforeach()

  foreach (qex_lib ${QEX_LIBRARIES})
    message(STATUS "Adding copy rule for ${qex_lib}")
	add_custom_command(TARGET ${PROJECT_NAME}
	  POST_BUILD
	  COMMAND
	    ${CMAKE_COMMAND} -E copy_if_different
		$<TARGET_FILE:${qex_lib}>
		$<TARGET_FILE_DIR:${PROJECT_NAME}>
	)
  endforeach()
  
  set(COMISO_OPENBLAS_DLL_LOC "${LIBIGL_INCLUDE_DIR}/../external/CoMISo/ext/OpenBLAS-v0.2.14-Win64-int64/bin")
  file(GLOB COMISO_OPENBLAS_DLLS ${COMISO_OPENBLAS_DLL_LOC}/*.dll)
  foreach (openblas_lib ${COMISO_OPENBLAS_DLLS})
	message(STATUS "Copying ${openblas_lib} to build directory.")
	file(COPY ${openblas_lib} DESTINATION ${PROJECT_BINARY_DIR})
  endforeach()
  
endif()

source_group(HeaderFiles FILES ${HEADER_FILES})
source_group(SourceFiles FILES ${SRC_FILES})
