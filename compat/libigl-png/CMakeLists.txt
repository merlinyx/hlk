# compat/libigl-png/CMakeLists.txt

add_library(hlk_libigl_png_compat INTERFACE)

target_include_directories(hlk_libigl_png_compat INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(hlk_libigl_png_compat INTERFACE
  igl::stb
  igl::opengl
)
