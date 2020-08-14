//
//  Util.hpp
//  Deform
//
//  Created by Kyle on 12/9/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef Util_hpp
#define Util_hpp

#include "Mesh.h"
#include "Matrix.h"

inline Vector3 toEigen(const OpenMesh::Vec3d& v)
{
    return Vector3(v[0], v[1], v[2]);
}

inline double clamp01(double v)
{
    return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
}

inline double clamp(double l, double h, double v)
{
    return v < l ? l : (v > h ? h : v);
}

inline bool is01(double v)
{
    return 0.0 <= v && v <= 1.0;
}


#endif /* Util_hpp */
