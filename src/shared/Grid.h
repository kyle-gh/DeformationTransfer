#pragma once

#include "Mesh.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Orthogonal_incremental_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/boost/iterator/counting_iterator.hpp>
#include <CGAL/Fuzzy_sphere.h>

#include <utility>
#include <list>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <memory>
#include <list>

class Grid
{
private:
    
    
    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef Kernel::Point_3 Point_d;
    
    struct PointContainerVector : public std::vector<std::pair<size_t, Point_d>>
    {
        typedef size_t key_type;
        typedef std::pair<key_type, Point_d> value_type;
        
        //static std::vector<value_type> _temp;
        
        PointContainerVector()
        : std::vector<std::pair<size_t, Point_d>>()
        {
            //_temp.resize(1);
        }
        
        inline iterator find(key_type k)
        {
            return begin() + k;
//            _temp[0].first = k;
//            _temp[0].second = (*this)[k];
//            return _temp.begin();
        }
        
        inline const_iterator find(key_type k) const
        {
            return cbegin() + k;
//            _temp[0].first = k;
//            _temp[0].second = (*this)[k];
//            return _temp.cbegin();
        }
        
        inline value_type::second_type& operator[](size_t k)
        {
            auto& p = *find(k);
            
            p.first = k;
            
            return p.second;
        }
        
        //const value_type* find(key_type k) const { _temp.first = k; _temp.second = (*this)[k]; return &_temp; }
        //const_iterator find(key_type k) const { return cbegin() + k; }
    };
    
    typedef std::map<size_t, Point_d> PointContainer;
    //typedef PointContainerVector PointContainer;
    
    typedef boost::const_associative_property_map<PointContainer> point_property_map;
    
    typedef CGAL::Search_traits_3<Kernel> TreeTraits_Base;
    typedef CGAL::Search_traits_adapter<std::size_t, point_property_map, TreeTraits_Base> TreeTraits;
    
    typedef CGAL::Orthogonal_incremental_neighbor_search<TreeTraits> IncrementalSearch;
    typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Search;
    
    typedef Search::Distance Distance;
    
    //typedef CGAL::Kd_tree<TreeTraits> Tree;
    typedef IncrementalSearch::Tree Tree;
    typedef Tree::Splitter Splitter;
    
    typedef CGAL::Fuzzy_sphere<TreeTraits> FuzzySphere;
    
    typedef std::unique_ptr<Tree> TreePtr;
    
    enum Target
    {
        Face,
        Vertex
    };
    
public:
    struct Result
    {
        int idx;
        double distanceSqr;
    };
    
    struct Context
    {
        std::vector<size_t> results;
    };
    
    typedef std::vector<Result> Results;
    
public:
	Grid();
    
    void setMesh(MeshPtr mesh, bool setBounds = true);
    
	void clear();
    
    void addFaces(bool useNormal = true, bool useUV = false);
    void addVertices(bool useNormal = true, bool useUV = false);
    
    bool getNearest(const OpenMesh::Vec3d& p, const OpenMesh::Vec3d& n, Result& result);
    
    bool getRange(const Mesh::Point& p, const Mesh::Point& n, float threshold, Results& results, Context* context = nullptr);
    
private:
    MeshPtr _mesh;
    
    TreePtr _tree;
    
    PointContainer _points;
    Context _context;
    
    std::vector<OpenMesh::Vec3d> _normals;
    
    Target _target;
    
    Distance _distance;
    
    point_property_map _ppMap;
    
    bool _useNormal;
    
    int _numAdded;
    
    void add(const Mesh::VertexHandle& v, const Mesh::Point& p);
    void add(const Mesh::FaceHandle& f, const Mesh::Point& p);
};

bool IsEqual(const Grid::Result& a, const Grid::Result& b);

bool CompareDistance(const Grid::Result& a, const Grid::Result& b);
