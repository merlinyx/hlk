# compat/libigl-comiso/CMakeLists.txt

add_library(hlk_libigl_comiso_compat INTERFACE)

target_include_directories(hlk_libigl_comiso_compat INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(hlk_libigl_comiso_compat INTERFACE
  igl::core
  Eigen3::Eigen
  CoMISo
)
