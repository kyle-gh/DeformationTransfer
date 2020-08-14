//
//  CorrespondenceUtil.h
//  Deform
//
//  Created by Kyle on 2/16/19.
//  Copyright © 2019 Kyle. All rights reserved.
//

#ifndef CorrespondenceUtil_h
#define CorrespondenceUtil_h

#include "../Correspondence.h"
#include "../Mesh.h"
#include "../Grid.h"

#include <memory>
#include <map>

class CorrespondenceUtil
{
public:
    typedef std::map<size_t, Mesh::Point> ConstraintMap;
    typedef std::shared_ptr<ConstraintMap> ConstraintMapPtr;
    
    static ConstraintMapPtr BuildConstraintsUV(MeshPtr source, CorrespondencePtr corr, MeshPtr target);
    
    static ConstraintMapPtr BuildConstraints(CorrespondencePtr corr, MeshPtr target);
    
    static void BuildVertex(MeshPtr mesh, Grid& grid, Correspondence& corr, float threshold = -1.0, int limit = -1, bool defaultToNearest = false);
    
    static void BuildFace(MeshPtr mesh, Grid& grid, Correspondence& corr, float threshold = -1.0, int limit = -1, bool defaultToNearest = false);
    
    static void BuildAdjacency(MeshPtr mesh, Correspondence& corr);
};

#endif /* CorrespondenceUtil_h */
