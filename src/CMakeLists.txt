set(sources
	z_axis_collider.cpp
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
	unity.cpp
	decoration.cpp
	pipe_erosian.cpp
	mesh_tesselator.cpp
	mesh_utils.cpp
	texture_utils.cpp
	normal_map_compressor.cpp
	river_mesh_builder.cpp
	tree_billboards.cpp
	logger.cpp
)

add_library(motu SHARED ${sources})
add_library(motu_static STATIC ${sources})