add_library(rp_library SHARED
  grid_mapping.cpp
  grid_map.cpp
  laser_scan.cpp
  dmap.cpp
  draw_helpers.cpp)

target_link_libraries(rp_library
    ${OpenCV_LIBS})

add_executable(grid_map_test
  grid_map_test.cpp
  )

target_link_libraries(grid_map_test
  rp_library
  ${OpenCV_LIBS}
  )

add_executable(dmap_test
  dmap_test.cpp)

target_link_libraries(dmap_test
  rp_library
  ${OpenCV_LIBS}
  )

