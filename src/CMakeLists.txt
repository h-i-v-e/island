set(sources
	island.cpp
	lodepng.cpp
	triangle3.cpp
	mesh.cpp
	mesh_edge_map.cpp
	mesh_triangle_map.cpp
	height_map.cpp
	raster.cpp
	simplex_noise.cpp
	terrain.cpp
	lake.cpp
	hydrolic_erosian.cpp
	sea_erosian.cpp
	rivers.cpp
	dalauney.cpp
	quantize_river.cpp
	unity.cpp
	decoration.cpp
	face_erosian.cpp
)

add_library(motu SHARED ${sources})
add_library(motu_static STATIC ${sources})