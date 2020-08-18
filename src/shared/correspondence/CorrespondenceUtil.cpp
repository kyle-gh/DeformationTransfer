//
//  CorrespondenceUtil.cpp
//  Deform
//
//  Created by Kyle on 2/16/19.
//  Copyright © 2019 Kyle. All rights reserved.
//

#include "CorrespondenceUtil.h"

#include "../Util.h"

#include <thread>
#include <mutex>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Barycentric_coordinates_2/Triangle_coordinates_2.h>
#include <CGAL/Triangle_2.h>

inline CorrespondenceUtil::ConstraintMapPtr MakeConstraintMap()
{
    return std::make_shared<CorrespondenceUtil::ConstraintMap>();
}

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point2;
typedef Kernel::Point_3 Point3;
typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<Kernel> TriangleCoordinates2;
typedef CGAL::Triangle_2<Kernel> Triangle2;

template <class T>
T fromUV(MeshPtr mesh, const Mesh::FaceHandle& face)
{
    Point2 p[3];
    
    int i = 0;
    
    for (auto vertIter = mesh->fv_begin(face), vertIterEnd = mesh->fv_end(face); vertIter != vertIterEnd; vertIter++)
    {
        const auto& tc = mesh->texcoord2D(*vertIter);
        
        p[i] = Point2(tc[0], tc[1]);
        
        i++;
    }
    
    return T(p[0], p[1], p[2]);
}

Mesh::Point barycenter(MeshPtr mesh, const Mesh::FaceHandle& face, const std::vector<double>& w, bool normal)
{
    Kernel::Point_3 p[3];
    
    int i = 0;
    
    for (auto vertIter = mesh->fv_begin(face), vertIterEnd = mesh->fv_end(face); vertIter != vertIterEnd; vertIter++)
    {
        const auto& v = normal ? mesh->normal(*vertIter) : mesh->point(*vertIter);
        
        p[i] = Kernel::Point_3(v[0], v[1], v[2]);
        
        i++;
    }
    
    auto c = CGAL::barycenter(p[0], w[0], p[1], w[1], p[2], w[2]);
    
    return Mesh::Point(c[0], c[1], c[2]);
}

inline bool isValid(const std::vector<double>& c)
{
    if (c.size() < 3)
        return false;
    
    const auto sum = c[0] + c[1] + c[2];
    
    return is01(c[0]) && is01(c[1]) && is01(c[2]) && sum == 1.0;
}

std::vector<Mesh::Point> uvTranslate(MeshPtr source, const std::vector<int> vertices, MeshPtr target)
{
    std::vector<Point2> uvs;
    uvs.resize(vertices.size());
    
    for (auto i = 0; i < vertices.size(); i++)
    {
        const auto& p = source->texcoord2D(source->vertex_handle(vertices[i]));
        
        uvs[i] = Point2(p[0], p[1]);
    }
    
    std::vector<Triangle2> triangles;
    triangles.resize(target->n_faces());
    
    for (auto faceIter = target->faces_begin(), faceEnd = target->faces_end(); faceIter != faceEnd; faceIter++)
    {
        const auto face = *faceIter;
        
        triangles[face.idx()] = fromUV<Triangle2>(target, face);
    }
    
    std::vector<Mesh::Point> points;
    points.resize(vertices.size());
    
    int vertexIndex = 0;
    std::mutex vertexLock;
    
    auto op =
    [&target, &uvs, &triangles, &points, &vertexIndex, &vertexLock]
    (int threadId)
    {
        std::vector<double> c;
        c.reserve(3);
        
        int vertex = 0;
        
        while(true)
        {
            vertexLock.lock();
            vertex = vertexIndex;
            vertexIndex++;
            vertexLock.unlock();
            
            if (vertex > points.size())
                break;
            
            const auto& uv = uvs[vertex];
            
            for (auto i = 0; i < triangles.size(); i++)
            {
                const auto& triangle = triangles[i];
                
                const auto side = triangle.oriented_side(uv);
                if (side == CGAL::ON_NEGATIVE_SIDE)
                    continue;
                
                const auto face = target->face_handle(i);
                
                c.clear();
                
                fromUV<TriangleCoordinates2>(target, face)(uv, c);
                
                if (!isValid(c))
                    continue;
                
                points[vertex] = barycenter(target, face, c, false);
            }
        }
    };
    
    if (vertices.size() < std::thread::hardware_concurrency())
        op(0);
    else
    {
        std::vector<std::thread> pool;
        for (auto i = 0; i < std::thread::hardware_concurrency(); i++)
            pool.push_back(std::thread(op, i));
        
        for (auto i = 0; i < pool.size(); i++)
            pool[i].join();
    }
    
    return points;
}

CorrespondenceUtil::ConstraintMapPtr CorrespondenceUtil::BuildConstraintsUV(MeshPtr source, CorrespondencePtr corr, MeshPtr target)
{
    auto map = MakeConstraintMap();
    
    Correspondence::PairsList pairs;
    corr->getPairs(pairs);
    
    std::vector<int> vertices;
    for (auto pair : pairs)
    {
        vertices.push_back(pair.first);
    }
    
    auto points = uvTranslate(source, vertices, target);
    
    for (auto i = 0; i < vertices.size(); i++)
    {
        (*map)[vertices[i]] = points[i];
    }
    
    return map;
}

CorrespondenceUtil::ConstraintMapPtr CorrespondenceUtil::BuildConstraints(CorrespondencePtr corr, MeshPtr target)
{
    auto map = MakeConstraintMap();
    
    Correspondence::PairsList pairs;
    corr->getPairs(pairs);
    
    for (auto pair : pairs)
    {
        const auto& p = target->point(target->vertex_handle(pair.second));
        
        (*map)[pair.first] = p;
    }
    
    return map;
}

unsigned int _Build(MeshPtr mesh, Search& search, Correspondence& corr, size_t numItems, std::function<bool(MeshPtr, int, Mesh::Point&, Mesh::Normal&)> get, float threshold = -1.0, int limit = -1, bool defaultToNearest = false, bool mulithread = true)
{
    corr.setSize(numItems);
    
    int items = 0;
    std::mutex itemLock;
    
    auto searchOp =
    [&mesh, &search, &corr, &get, threshold, limit, defaultToNearest, &items, numItems, &itemLock]
    (int threadId)
    {
        int item = 0;
        
        Mesh::Point p;
        Mesh::Normal n;
        
        Search::Results results;
        Search::Result nearest;
        
        Search::Context context;
        
        bool nearestSearch = threshold <= 0 && limit <= 1;
        
        while(true)
        {
            itemLock.lock();
            item = items;
            items++;
            itemLock.unlock();
            
            if (item >= numItems)
                break;
            
            if (!get(mesh, item, p, n))
                break;
            
            if (isZero(n))
            {
                std::cerr << "**Invalid Normal - Item " << item << std::endl;
                continue;
            }
            
            if (nearestSearch)
            {
                if (search.getNearest(p, n, nearest))
                    corr.add(item, nearest.idx);
            }
            else
            {
                if (search.getRange(p, n, threshold, results, &context))
                {
                    std::sort(results.begin(), results.end(), CompareDistance);
                    
                    auto size = (int)results.size();
                    if (limit > 0)
                        size = std::min(limit, size);
                    
                    auto& c = corr.get(item);
                    for (int i = 0; i < limit; i++)
                        c.push_back(results[i].idx);
                }
                else
                {
                    if (defaultToNearest)
                        if (search.getNearest(p, n, nearest))
                            corr.add(item, nearest.idx);
                }
            }
        }
    };
    
    if (mulithread)
    {
        std::vector<std::thread> pool;
        
        for (auto i = 0; i < std::thread::hardware_concurrency(); i++)
            pool.push_back(std::thread(searchOp, i));
        
        for (auto i = 0; i < pool.size(); i++)
            pool[i].join();
    }
    else
    {
        searchOp(0);
    }
    
    return 1;
}

void CorrespondenceUtil::BuildVertex(MeshPtr mesh, Search& search, Correspondence& corr, float threshold, int limit, bool defaultToNearest)
{
    auto get =
    []
    (MeshPtr mesh, int idx, Mesh::Point& p, Mesh::Normal& n)
    {
        const auto vert = mesh->vertex_handle(idx);
        
        p = mesh->point(vert);
        n = mesh->normal(vert);
        
        return true;
    };
    
    _Build(mesh, search, corr, mesh->n_vertices(), get, threshold, limit, defaultToNearest, false);
}

void CorrespondenceUtil::BuildFace(MeshPtr mesh, Search& search, Correspondence& corr, float threshold, int limit, bool defaultToNearest)
{
    auto get =
    []
    (MeshPtr mesh, int idx, Mesh::Point& p, Mesh::Normal& n)
    {
        const auto face = mesh->face_handle(idx);
        
        p = mesh->calc_face_centroid(face);
        n = mesh->calc_face_normal(face);
        
        return true;
    };
    
    _Build(mesh, search, corr, mesh->n_faces(), get, threshold, limit, defaultToNearest);
}

void CorrespondenceUtil::BuildAdjacency(MeshPtr mesh, Correspondence& corr)
{
    corr.setSize(mesh->n_faces());
    
    for (auto face_iter = mesh->faces_begin(), face_end = mesh->faces_end(); face_iter != face_end; face_iter++)
    {
        const auto face = *face_iter;
        
        for (auto adj_iter = mesh->ff_begin(face), adj_end = mesh->ff_end(face); adj_iter != adj_end; adj_iter++)
        {
            const auto adj_face = *adj_iter;
            
            corr.add(face.idx(), adj_face.idx());
        }
        
        //        auto& c = corr.get(face.idx());
        //
        //        if (c.size() == 3)
        //        {
        //            std::swap(c[0], c[1]);
        //            std::swap(c[1], c[2]);
        //        }
    }
}

/*
MeshPtr morphTo(MeshPtr source, MeshPtr target)
{
    auto morph = MakeMesh(source);
    
    std::vector<Triangle2> triangles;
    triangles.resize(target->n_faces());
    
    std::vector<double> c;
    c.reserve(3);
    
    for (auto faceIter = target->faces_begin(), faceEnd = target->faces_end(); faceIter != faceEnd; faceIter++)
    {
        const auto face = *faceIter;
        
        triangles[face.idx()] = uvTriangle(target, face);
    }
    
    std::cout << "Built Triangles: " << triangles.size() << std::endl;
    
    std::cout << "Genereting new vertices" << std::endl;
    
    size_t numVertices = 0;
    
    for (auto vertIter = source->vertices_begin(), vertEnd = source->vertices_end(); vertIter != vertEnd; vertIter++)
    {
        const auto vert = *vertIter;
        
        if (vert.idx() % 100 == 0)
            std::cout << "Vert: " << vert.idx();
        
        const auto& tc = source->texcoord2D(vert);
        const auto uv = Point2(tc[0], tc[1]);
        
        for (auto i = 0; i < triangles.size(); i++)
        {
            const auto& triangle = triangles[i];
            if (triangle.is_degenerate())
                continue;
            
            const auto side = triangle.oriented_side(uv);
            if (side == CGAL::ON_NEGATIVE_SIDE)
                continue;
            
            c.clear();
            uvBarycentric(target, target->face_handle(i))(uv, c);
            
            if (!isValid(c))
                continue;
            
            const auto np = barycenterPosition(target, target->face_handle(i), c);
            const auto nn = barycenterNormal(target, target->face_handle(i), c);
            
            morph->point(morph->vertex_handle(vert.idx())) = np;
            morph->set_normal(morph->vertex_handle(vert.idx()), nn);
            
            if (vert.idx() % 100 == 0)
                std::cout << " -> " << i << std::endl;
            
            numVertices++;
            
            break;
        }
    }
    
    if (numVertices != morph->n_vertices())
    {
        std::cout << "Vertices Morphed: " << numVertices << "/" << morph->n_vertices() << std::endl;
    }
    
    return morph;
}
*/
