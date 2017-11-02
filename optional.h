//
//  optional.h
//  World Maker
//
//  Created by Jerome Johnson on 30/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef optional_h
#define optional_h

namespace worldmaker{
    template <bool applyFirst, class First, class Second>
    struct Optional;
    
    template <class First, class Second>
    struct Optional<true, First, Second> {
        typedef First value;
    };
    
    template <class First, class Second>
    struct Optional<false, First, Second>{
        typedef Second value;
    };
}

#endif /* optional_h */
