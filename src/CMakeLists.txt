set(sources
	island.cpp
	lodepng.cpp
	mesh.cpp
	mesh_edge_map.cpp
	mesh_triangle_map.cpp
	raster.cpp
	simplex_noise.cpp
	terrain.cpp
	hydrolic_erosian.cpp
	sea_erosian.cpp
	rivers.cpp
	dalauney.cpp
)

add_library(motu SHARED ${sources} unity.cpp)
add_library(motu_static STATIC ${sources})
#set_target_properties(motu PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/libs")