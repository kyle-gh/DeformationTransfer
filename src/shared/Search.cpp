#include "Search.h"

//std::vector<Search::PointContainerVector::value_type> Search::PointContainerVector::_temp;

inline Mesh::Point uvCentroid(MeshPtr mesh, const Mesh::FaceHandle& face)
{
    auto centroid = Mesh::Point(0, 0, 0);
    auto numVertices = 0;
    
    for (auto vertIter = mesh->fv_begin(face), vertIterEnd = mesh->fv_end(face); vertIter != vertIterEnd; vertIter++)
    {
        const auto& tc = mesh->texcoord2D(*vertIter);
        centroid[0] += tc[0];
        centroid[1] += tc[1];
        numVertices++;
    }
    
    if (numVertices > 1)
    {
        centroid[0] /= numVertices;
        centroid[1] /= numVertices;
    }
    
    return centroid;
}

inline Mesh::Point uv(MeshPtr mesh, const Mesh::VertexHandle& vert)
{
    const auto& tc = mesh->texcoord2D(vert);
    
    return Mesh::Point(tc[0], tc[1], 0);
}

Search::Search()
: _ppMap(_points)
, _distance(_ppMap)
, _tree(nullptr)
{
}

void Search::setMesh(MeshPtr mesh, bool setBounds)
{
    clear();
    
    _mesh = mesh;
}

void Search::clear()
{
    _tree = nullptr;//.clear();
    _points.clear();
    _normals.clear();
    _numAdded = 0;
}

void Search::addFaces(bool useNormal, bool useUV)
{
    clear();
    
    if (useNormal)
    {
        _mesh->update_face_normals();
        _normals.resize(_mesh->n_faces());
    }
    
    //_points.resize(_mesh->n_faces());
    
    Mesh::Point centroid;
    Mesh::Point normal;
    
    for(auto face_iter = _mesh->faces_begin(), face_end = _mesh->faces_end(); face_iter != face_end; face_iter++)
    {
        auto face = *face_iter;
        
        if (useNormal)
        {
            normal = _mesh->calc_face_normal(face);
            
            if (isZero(normal))
            {
                std::cerr << "Invalid Normal: Face " << face.idx() << " - " << normal << std::endl;
                continue;
            }
            
            _normals[face.idx()] = normal;
        }
        
        if (useUV)
            centroid = uvCentroid(_mesh, face);
        else
            centroid = _mesh->calc_face_centroid(face);
        
        add(face, centroid);
    }
    
    _target = Face;
    
    _tree = std::make_unique<Tree>(Splitter(), TreeTraits(_ppMap));
    _tree->insert(boost::counting_iterator<std::size_t>(0), boost::counting_iterator<std::size_t>(_points.size()));
    _tree->build();
    
    _distance = Distance(_ppMap);
    
    _useNormal = useNormal;
    
    std::cout << "Search - Faces: " << _points.size() << std::endl;
}

void Search::addVertices(bool useNormal, bool useUV)
{
    clear();
    
    Mesh::Point point;
    Mesh::Point normal;
    
    if (useNormal)
        _normals.resize(_mesh->n_vertices());
    
    //_points.resize(_mesh->n_vertices());
    
    for(auto vert_iter = _mesh->vertices_begin(), vert_end = _mesh->vertices_end(); vert_iter != vert_end; vert_iter++)
    {
        const auto vert = *vert_iter;
        
        if (useNormal)
        {
            normal = _mesh->normal(vert);
            
            if (isZero(normal))
            {
                std::cerr << "**Invalid Normal: Vertex " << vert.idx() << " - " << normal << std::endl;
                continue;
            }
            
            _normals[vert.idx()] = normal;
        }
        
        if (useUV)
            point = uv(_mesh, vert);
        else
            point = _mesh->point(vert);
        
        add(vert, point);
    }
    
    _target = Vertex;
    
    _tree = std::make_unique<Tree>(Splitter(), TreeTraits(_ppMap));
    _tree->insert(boost::counting_iterator<std::size_t>(0), boost::counting_iterator<std::size_t>(_points.size()));
    _tree->build();
    
    _distance = Distance(_ppMap);
    
    _useNormal = useNormal;
    
    std::cout << "Search - Vertices: " << _points.size() << std::endl;
}

void Search::add(const Mesh::VertexHandle& v, const Mesh::Point& p)
{
    _points[v.idx()] = Point_d(p[0], p[1], p[2]);
    _numAdded++;
}

void Search::add(const Mesh::FaceHandle& f, const Mesh::Point& p)
{
    _points[f.idx()] = Point_d(p[0], p[1], p[2]);
    _numAdded++;
}

bool Search::getNearest(const OpenMesh::Vec3d& p, const OpenMesh::Vec3d& n, Result& result)
{
    result.idx = -1;
    result.distanceSqr = __DBL_MAX__;
    
    if (_useNormal)
        if (isZero(n))
            return false;
    
    const Point_d query(p[0], p[1], p[2]);
    
    IncrementalSearch search(*_tree, query, 0.0, true, _distance);
    
    for(auto it = search.begin(); it != search.end(); ++it)
    {
        if (_useNormal)
        {
            const auto& normal = _normals[it->first];
            
            if ((normal | n) <= 0)
                continue;
        }
        
        result.idx = (int)it->first;
        result.distanceSqr = it->second;
        
        break;
    }
    
    return result.idx != -1;
}

bool Search::getRange(const Mesh::Point& p, const Mesh::Point& n, float threshold, Results& results, Context* context)
{
    results.clear();
    
    if (_useNormal)
        if (isZero(n))
            return false;
    
    // Squared-distance/threshold
    //const auto thresholdSqr = threshold * threshold;
    
    if (context == nullptr)
        context = &_context;
    
    context->results.clear();
    
    const Point_d query(p[0], p[1], p[2]);
    
    FuzzySphere sphere(query, threshold, 0.0, TreeTraits(_ppMap));
    
    _tree->search(std::back_inserter(context->results), sphere);
    
    for (auto result : context->results)
    {
        if (_useNormal)
        {
            const auto& normal = _normals[result];
            
            if ((normal | n) <= 0)
                continue;
        }
        
        if (result >= _points.size())
            continue;
        
        const auto& r = _points[result];
        auto distanceSqr = 0.0;
        for (auto i = 0; i < 3; i++)
            distanceSqr += (r[i] * query[i]);
        
        results.push_back(Result{(int)result, distanceSqr});
    }
    
    //Search search(*_tree, query, 0.0, true, _distance);
    
//    for(auto it = search.begin(); it != search.end(); ++it)
//    {
//        if (it->second >= thresholdSqr)
//            break;
//
//        if (_useNormal)
//        {
//            const auto& normal = _normals[it->first];
//
//            if ((normal | n) <= 0)
//                continue;
//        }
//
//        results.push_back(Result{(int)it->first, it->second});
//    }
    
    return !results.empty();
}

bool IsEqual(const Search::Result& a, const Search::Result& b)
{
    return a.idx == b.idx;
}

bool CompareDistance(const Search::Result& a, const Search::Result& b)
{
    return (a.distanceSqr < b.distanceSqr);
}
