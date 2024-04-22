add_executable(dmap_localizer_test
  dmap_localizer.cpp
  dmap_localizer_test.cpp)

target_link_libraries(dmap_localizer_test
  rp_library)

add_executable(dmap_grid_localizer_test
  dmap_localizer.cpp
  dmap_grid_localizer_test.cpp)

target_link_libraries(dmap_grid_localizer_test
  rp_library)
