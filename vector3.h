//
//  vector3.h
//  World Maker
//
//  Created by Jerome Johnson on 28/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef vector3_h
#define vector3_h

namespace worldmaker{
    struct Vector3{
        Vector3(float x, float y, float z) : x(x), y(y), z(z){}

        float x, y, z;
    };
}

#endif /* vector3_h */
