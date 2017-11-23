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

namespace worldmaker{
    struct Mesh{
        typedef std::vector<Vector3> Vertices;
        typedef std::vector<Vector3> Normals;
        typedef std::vector<Vector2> TextureCoordinates;
        typedef std::vector<int> Triangles;
        
        Vertices vertices;
        Normals normals;
        TextureCoordinates textureCoordinates;
        Triangles triangles;
        
        void decimate(int vertices);
    };
}

#endif /* mesh_h */
