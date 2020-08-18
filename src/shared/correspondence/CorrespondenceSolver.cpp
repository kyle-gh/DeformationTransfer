//
//  CorrespondenceSolver.cpp
//  Deform
//
//  Created by Kyle on 12/7/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "CorrespondenceSolver.h"

#include "CorrespondenceUtil.h"

#include "../Timing.h"

#include <iostream>
#include <fstream>

#define INVALID ((unsigned int)-1)

CorrespondenceSolver::CorrespondenceSolver()
: SolverBase()
{
    _weights = {
        {1.0, 0.1, 0},
        {1.0, 0.1, 1},
        {1.0, 0.1, 30},
        {1.0, 0.1, 40},
        {1.0, 0.1, 50}
    };
    
//    _weights = {
//        {1.0, 0.001, 0},
//        {1.0, 0.001, 1},
//        {1.0, 0.001, 1666.0},
//        {1.0, 0.001, 3333.0},
//        {1.0, 0.001, 5000.0}
//    };
    
    _maxCorrespondence = 3;
}

CorrespondenceSolver::~CorrespondenceSolver()
{
}

bool CorrespondenceSolver::setSourceReference(MeshPtr mesh)
{
    std::cout
        << "Source Reference" << std::endl
        << "\tVertices: " << mesh->n_vertices() << std::endl
        << "\tTriangles: " << mesh->n_faces() << std::endl;
    
    // Copy mesh, destructive process to follow.
    _source = MakeMesh(mesh);
    
    CorrespondenceUtil::BuildAdjacency(_source, _faceAdjacency);
    
    _invSurface.resize(_source->n_faces());
    
    _es.resize(_source->n_faces());
    _cs.resize(_source->n_faces());
    
    return true;
}

bool CorrespondenceSolver::setTargetReference(MeshPtr mesh)
{
    std::cout
        << "Target Reference" << std::endl
        << "\tVertices: " << mesh->n_vertices() << std::endl
        << "\tTriangles: " << mesh->n_faces() << std::endl;
    
    _target = mesh;
    
    _grid.setMesh(_target);
    _grid.addVertices();
    
    return true;
}

bool CorrespondenceSolver::setVertexConstraints(ConstraintMapPtr map)
{
    _anchorMap = map;
    
    if (_anchorMap == nullptr)
        return false;
    
    std::cout
        << "Vertex Constraints:" << std::endl
        << "\tConstrained: " << _anchorMap->size() << std::endl
        << "\tFree: " << (_source->n_vertices() - _anchorMap->size()) << std::endl;
    
    _vertexMap.resize(_source->n_vertices());
    _freeVertices = 0;
    
    for (auto i = 0; i < _source->n_vertices(); i++)
    {
        if (_anchorMap->find(i) == _anchorMap->end())
        {
            _vertexMap[i] = _freeVertices;
            _freeVertices++;
        }
        else
        {
            _vertexMap[i] = INVALID;
        }
    }
    
    return true;
}

void CorrespondenceSolver::setWeights(const std::vector<Weights>& weights)
{
    _weights = weights;
}

void CorrespondenceSolver::setStepCallback(StepCallback callback)
{
    _stepCallback = callback;
}

bool CorrespondenceSolver::resolve()
{
    for (auto step = 0; step < _weights.size(); step++)
    {
        const auto& weights = _weights[step];
        
        std::cout << std::endl
        << "Resolve Step [" << step << "]" << std::endl
            << "\tSmoothness:" << weights.smooth << std::endl
            << "\tIdentity:" << weights.identity << std::endl
            << "\tClosest:" << weights.closest << std::endl;
        
        resetSolve();
        
        TIMER_START(ConstructProblem);
        
        if (weights.closest == 0)
            solveSI(weights);
        else
            solveSIC(weights);
        
        _A.setFromTriplets(_m.begin(), _m.end());
        
        TIMER_END(ConstructProblem);
        
        TIMER_START(Solve);
        
        solve(_A, _c, _x);
        
        TIMER_END(Solve);
        
        TIMER_START(CopyToMesh);
        
        copyTo(_x, *_source);
        
        TIMER_END(CopyToMesh);
        
        if (_stepCallback)
            _stepCallback(step, _source);
    }
    
    if (_weights.empty())
        std::cout << "Solving skipped..." << std::endl;
    
    constructCorrespondence();
    
    return true;
}

const Correspondence& CorrespondenceSolver::faceCorrespondence() const
{
    return _faceCorr;
}

void CorrespondenceSolver::resetSolve()
{
    _row = 0;
}

void CorrespondenceSolver::solveSI(const Weights& weights)
{
    const auto rowSize = 9 * (_faceAdjacency.numPairs() + _source->n_faces());
    const auto colSize = 3 * (_freeVertices + _source->n_faces());
    
    std::cout
        << "Solving Smoothness+Identity" << std::endl
        << "\tProblem Size: " << rowSize << " x " << colSize << " :: " << colSize << " x 1" << std::endl;
    
    _m.reserve(rowSize * colSize);
    _m.clear();
    
    _A.resize(rowSize, colSize);
    _A.setZero();
    
    _c.resize(rowSize, 1);
    _c.setZero();
    
    constructInvSurfaces(*_source, _invSurface);
    constructECs(*_source, *_target);
    
    // Es + Ei
    appendSmoothness(weights.smooth, _m, _c);
    appendIdentity(weights.identity, _m, _c);
}

void CorrespondenceSolver::solveSIC(const Weights& weights)
{
    const auto rowSize = 9 * (_faceAdjacency.numPairs() + _source->n_faces()) + (3 * _freeVertices);
    const auto colSize = 3 * (_freeVertices + _source->n_faces());
    
    std::cout
        << "Solving Smoothness+Identity+Closest" << std::endl
        << "\tProblem Size: " << rowSize << " x " << colSize << " :: " << colSize << " x 1" << std::endl;
    
    _m.reserve(rowSize * colSize);
    _m.clear();
    
    _A.resize(rowSize, colSize);
    _A.setZero();
    
    _c.resize(rowSize, 1);
    _c.setZero();
    
    // from Phase 1
    constructInvSurfaces(*_source, _invSurface);
    constructECs(*_source, *_target);
    
    // Es + Ei
    appendSmoothness(weights.smooth, _m, _c);
    appendIdentity(weights.identity, _m, _c);
    
    // Ec
    CorrespondenceUtil::BuildVertex(_source, _grid, _nearestCorr);
    appendClosest(_nearestCorr, weights.closest, _m, _c);
}

float MeshThreshold(MeshPtr a)
{
    Mesh::Point min, max;
    
    Bounds(*a, min, max);
    
    auto diff = max - min;
    
    return std::sqrt(4 * diff.sqrnorm() / a->n_vertices());
}

float MeshThreshold(MeshPtr a, MeshPtr b)
{
    return std::min(MeshThreshold(a), MeshThreshold(b));
}

void CorrespondenceSolver::constructCorrespondence()
{
    std::cout
        << std::endl
        << "Constructing Face Correspondences" << std::endl;
    
    auto threshold = MeshThreshold(_source, _target);
    
    std::cout
    << "\t#Faces: " << _source->n_faces() << std::endl
    << "\tThreshold: " << threshold << std::endl;
    
    _grid.setMesh(_source);
    _grid.addFaces();
    
    CorrespondenceUtil::BuildFace(_target, _grid,  _faceCorr, threshold, _maxCorrespondence, true);
    
    std::vector<unsigned int> noCorrespondence;
    
    for (auto i = 0; i < _target->n_faces(); i++)
    {
        if (_faceCorr.has(i))
            continue;
        
        noCorrespondence.push_back(i);
    }
    
    std::cout << "\tTotal: " << (_target->n_faces() - noCorrespondence.size()) << std::endl;
    
    if (!noCorrespondence.empty())
    {
        std::cout
        << "Missing Correspondence: "
        << noCorrespondence.size() << " / " << _target->n_faces()
        << " (" << ((double)noCorrespondence.size() / (double)_target->n_faces()) * 100.0 << "%)" << std::endl;
        
//        for (auto i = 0; i < noCorrespondence.size(); i++)
//            std::cout << "\t" << noCorrespondence[i] << std::endl;
    }
}

void CorrespondenceSolver::appendSmoothness(double weight, TripletList& m, MatrixX& c)
{
    for (int tri = 0; tri < _source->n_faces(); tri++)
    {
        appendSmoothness(_source->face_handle(tri), weight, m, c);
    }
}

void CorrespondenceSolver::appendSmoothness(const Mesh::FaceHandle& face, double weight, TripletList& m, MatrixX& c)
{
    const auto& adjacent = _faceAdjacency.get(face.idx());
    
    for (auto adjIdx : adjacent)
    {
        auto row = _row;
        appendEC(face, weight, m, c);
        
        _row = row;
        appendEC(_source->face_handle(adjIdx), -weight, m, c);
    }
}

void CorrespondenceSolver::appendIdentity(double weight, TripletList& m, MatrixX& c)
{
    for (int tri = 0; tri < _source->n_faces(); tri++)
    {
        appendIdentity(_source->face_handle(tri), weight, m, c);
    }
}

void CorrespondenceSolver::appendIdentity(const Mesh::FaceHandle& face, double weight, TripletList& m, MatrixX& c)
{
    const auto& face_e = _es[face.idx()];
    const auto& face_c = _cs[face.idx()];
    
    appendEC(face, face_e, face_c + C_Identity, weight, m, c);
}

void CorrespondenceSolver::appendClosest(const Correspondence& nearest, double weight, TripletList& m, MatrixX& c)
{
    for (int vert = 0; vert < _source->n_vertices(); vert++)
    {
        if (_anchorMap->count(vert) > 0)
            continue;
        
        appendClosest(_source->vertex_handle(vert), nearest, weight, m, c);
    }
}

void CorrespondenceSolver::appendClosest(const Mesh::VertexHandle& vert, const Correspondence& nearest, double weight, TripletList& m, MatrixX& c)
{
    const auto idx = (int)vertexIndex(vert);
    
    const auto nearestVert = _target->vertex_handle(nearest.get(vert.idx())[0]);
    const auto nearestP = _target->point(nearestVert);
    
    m.push_back(Triplet{_row, idx, weight});
    c(_row, 0) += weight * nearestP[0];
    _row++;
    
    m.push_back(Triplet{_row, idx + 1, weight});
    c(_row, 0) += weight * nearestP[1];
    _row++;
    
    m.push_back(Triplet{_row, idx + 2, weight});
    c(_row, 0) += weight * nearestP[2];
    _row++;
}

void CorrespondenceSolver::appendEC(const Mesh::FaceHandle& face, double weight, TripletList& m, MatrixX& c)
{
    const auto& face_e = _es[face.idx()];
    const auto& face_c = _cs[face.idx()];
    
    appendEC(face, face_e, face_c, weight, m, c);
}

void CorrespondenceSolver::appendEC(const Mesh::FaceHandle& face, const Matrix9x4& face_e, const Matrix9x1& face_c, double weight, TripletList& m, MatrixX& c)
{
    unsigned int vertIndices[4];
    vertexIndices(*_source, face, vertIndices);
    
    int l_row = 0;
    
    if (!face_c.isZero())
    {
        c.block<9, 1>(_row, 0) += (weight * face_c);
    }
    
    for (int coord = 0; coord < 3; coord++)
    {
        for (int eqn = 0; eqn < 3; eqn++, _row++, l_row++)
        {
            for (int vert = 0; vert < 4; vert++)
            {
                const int idx = (int)vertIndices[vert];
                if (idx != INVALID)
                {
                    m.push_back(Triplet{_row, idx + coord, weight * face_e(l_row, vert)});
                }
            }
        }
    }
}

void CorrespondenceSolver::constructECs(const Mesh& source, const Mesh& target)
{
    for (int idx = 0; idx < source.n_faces(); idx++)
    {
        constructEC(source, target, source.face_handle(idx), _es[idx], _cs[idx]);
    }
}

void CorrespondenceSolver::constructEC(const Mesh& source, const Mesh& target, const Mesh::FaceHandle& face, Matrix9x4& e, Matrix9x1& c)
{
    const auto& invSurface = _invSurface[face.idx()];
    
    e.setZero();
    c.setZero();

    Eigen::Matrix<double, 3, 4> is;
    for (int i = 0; i < 3; i++)
    {
        const auto col = invSurface.col(i);
        is(i, 0) = -col.sum();
        is(i, 1) = col(0, 0);
        is(i, 2) = col(1, 0);
        is(i, 3) = col(2, 0);
    }

    e.block<3, 4>(0, 0) = is;
    e.block<3, 4>(3, 0) = is;
    e.block<3, 4>(6, 0) = is;
    
    Matrix9x1 v;
    
    auto colIdx = 0;
    for (auto vert_iter = source.cfv_begin(face), vert_end = source.cfv_end(face); vert_iter != vert_end; colIdx++, vert_iter++)
    {
        const auto vert = *vert_iter;
        
        auto findIter = _anchorMap->find(vert.idx());
        if (findIter != _anchorMap->end())
        {
            auto col = e.col(colIdx);
            
            const auto& p = findIter->second;
            
            v << p[0], p[0], p[0], p[1], p[1], p[1], p[2], p[2], p[2];
            
            for (int i = 0; i < 9; i++)
                v[i] *= col(i);
            
            col.setZero();
            
            c -= v;
        }
    }
}

void CorrespondenceSolver::constructInvSurfaces(const Mesh& mesh, std::vector<Matrix3x3>& invSurfaces)
{
    invSurfaces.resize(mesh.n_faces());
    
    for (auto face_iter = mesh.faces_begin(), face_end = mesh.faces_end(); face_iter != face_end; face_iter++)
    {
        auto face = *face_iter;
        
        calculateInvSurface(mesh, face, invSurfaces[face.idx()]);
    }
}

bool CorrespondenceSolver::solve(SparseMatrix& a, MatrixX& c, MatrixX& x)
{    
    a.makeCompressed();
    
    auto at = a.transpose();
    auto ata = at * a;
    auto atc = at * c;
    
    _solver.compute(ata);
    
    if( !checkSolverError())
        return false;
    
    x = _solver.solve(atc);
    
    return checkSolverError();
}

void CorrespondenceSolver::vertexIndices(const Mesh& mesh, const Mesh::FaceHandle& face, unsigned int indices[])
{
    // v1, v2, v3
    auto vert_index = 0;
    for (auto vert_iter = mesh.cfv_begin(face), vert_end = mesh.cfv_end(face); vert_iter != vert_end; vert_iter++)
    {
        const auto vert = *vert_iter;
        
        const auto idx = vertexIndex(vert);
        
        indices[vert_index] = idx;
        
        vert_index++;
    }
    
    // v4
    indices[vert_index] = (_freeVertices + face.idx()) * 3;
}

unsigned int CorrespondenceSolver::vertexIndex(const Mesh::VertexHandle& vert) const
{
    return vertexIndex(vert.idx());
}

unsigned int CorrespondenceSolver::vertexIndex(unsigned int vertIdx) const
{
    const auto idx = _vertexMap[vertIdx];
    return idx == INVALID ? idx : idx * 3;
}

void CorrespondenceSolver::copyTo(const MatrixX& x, Mesh& mesh)
{
    int logVerts = 3;
    
    std::cout << "Applying Transformation: " << std::endl;
    
    for (auto vert_iter = mesh.vertices_begin(), vert_end = mesh.vertices_end(); vert_iter != vert_end; vert_iter++)
    {
        const auto vert = *vert_iter;
        
        auto findIter = _anchorMap->find(vert.idx());
        if (findIter != _anchorMap->end())
        {
            mesh.point(vert) = findIter->second;
        }
        else
        {
            const auto idx = vertexIndex(vert);
            const auto p = Mesh::Point(x(idx, 0), x(idx + 1, 0), x(idx + 2, 0));
            mesh.point(vert) = p;
        }
        
        if (logVerts > 0)
        {
            std::cout << "\tVert " << vert.idx() << ": " << mesh.point(vert) << std::endl;
            logVerts--;
        }
    }
}
