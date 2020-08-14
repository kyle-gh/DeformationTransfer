//
//  Mesh.h
//  Deform
//
//  Created by Kyle on 11/18/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef Mesh_h
#define Mesh_h

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <memory>

#include <iostream>

// OpenMesh Mesh Traits
// Use Double-based structures
struct DoubleTraits : OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
};

typedef OpenMesh::TriMesh_ArrayKernelT<DoubleTraits> Mesh;

typedef std::shared_ptr<Mesh> MeshPtr;

inline MeshPtr MakeMesh(MeshPtr copy = nullptr)
{
    auto mesh = std::make_shared<Mesh>();
    if (copy != nullptr)
        *mesh = *copy;
    
    return mesh;
}

inline bool isNan(const OpenMesh::Vec3d& v)
{
    return std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2]);
}

inline bool isZero(const OpenMesh::Vec3d& v)
{
    return v[0] == 0.0 && v[1] == 0.0 && v[2] == 0.0;
}

inline void FaceVertices(const Mesh& mesh, const Mesh::FaceHandle& face, Mesh::VertexHandle vertices[])
{
    size_t i = 0;
    for (auto vertIter = mesh.cfv_begin(face), vertEnd = mesh.cfv_end(face); i < 3 && vertIter != vertEnd; vertIter++, i++)
    {
        vertices[i] = (*vertIter);
    }
}

inline void Adjacent(const Mesh& mesh, const Mesh::FaceHandle& face, Mesh::FaceHandle adjacent[])
{
    size_t i = 0;
    for (auto adj_iter = mesh.cff_begin(face), adj_end = mesh.cff_end(face); i < 3 && adj_iter != adj_end; i++, adj_iter++)
    {
        adjacent[i] = *adj_iter;
    }
}

inline void Bounds(const Mesh& mesh, OpenMesh::Vec3d& min, OpenMesh::Vec3d& max)
{
    min = mesh.point(mesh.vertex_handle(0));
    max = min;
    
    for (auto vert_iter = mesh.vertices_begin(), vert_end = mesh.vertices_end(); vert_iter != vert_end; vert_iter++)
    {
        const auto& p = mesh.point(*vert_iter);
        
        min.minimize(p);
        max.maximize(p);
    }
}

MeshPtr ReadMesh(const std::string& path, bool exitOnFail = false);

bool WriteMesh(const std::string& path, MeshPtr mesh);

#endif /* Mesh_h */
