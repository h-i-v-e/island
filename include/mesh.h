//
//  mesh.h
//  World Maker
//
//  Created by Jerome Johnson on 11/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef mesh_h
#define mesh_h

#include <vector>
#include "vector3.h"
#include "grid.h"


namespace motu{
    struct Mesh{
        typedef std::vector<Vector3> Vertices;
        typedef std::vector<Vector3> Normals;
        typedef std::vector<size_t> Triangles;

		struct VertexAndNormal {
			Vector3 vertex, normal;
		};
        
        Vertices vertices;
        Normals normals;
        Triangles triangles;

		void calculateNormals();

		void smooth();
        
        void decimate(int vertices);

		void rasterize(Grid<VertexAndNormal> &) const;

		void rasterize(Grid<Vector3> &) const;
    };
}

#endif /* mesh_h */
