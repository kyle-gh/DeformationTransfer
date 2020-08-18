//
//  TransferSolver.cpp
//  Deform
//
//  Created by Kyle on 11/18/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "TransferSolver.h"

#include "../Timing.h"

#include "../Util.h"

#include <iostream>
#include <algorithm>

#define INVALID ((unsigned int)-1)

TransferSolver::TransferSolver()
: SolverBase()
{
}

TransferSolver::~TransferSolver()
{
}

bool TransferSolver::setSourceReference(MeshPtr mesh)
{
    std::cout
        << "Source Reference" << std::endl
        << "\tVertices: " << mesh->n_vertices() << std::endl
        << "\tTriangles: " << mesh->n_faces() << std::endl;
    
    TIMER_START(CalculateInvVr);
    
    _invVr.resize(mesh->n_faces());
    
    for (auto face_iter = mesh->faces_begin(), face_end = mesh->faces_end(); face_iter != face_end; face_iter++)
    {
        const auto face = *face_iter;
        
        calculateInvSurface(*mesh, face, _invVr[face.idx()]);
    }
    
    TIMER_END(CalculateInvVr);
    
    constructQ();
    
    return true;
}

bool TransferSolver::setEmptySourceReference(MeshPtr mesh)
{
    std::cout
    << "Source Reference (Empty)" << std::endl
    << "\tVertices: " << mesh->n_vertices() << std::endl
    << "\tTriangles: " << mesh->n_faces() << std::endl;
    
    TIMER_START(CalculateInvVr);
    
    _invVr.resize(mesh->n_faces());
    
    for (auto face_iter = mesh->faces_begin(), face_end = mesh->faces_end(); face_iter != face_end; face_iter++)
    {
        const auto face = *face_iter;
        
        _invVr[face.idx()] = Matrix3x3::Identity();
    }
    
    TIMER_END(CalculateInvVr);
    
    constructQ();
    
    return true;
}

bool TransferSolver::setSourceDeform(MeshPtr mesh)
{
    std::cout
        << "Source Deform" << std::endl
        << "\tVertices: " << mesh->n_vertices() << std::endl
        << "\tTriangles: " << mesh->n_faces() << std::endl;
    
    TIMER_START(CalculateVd);
    
    _Vd.resize(mesh->n_faces());
    
    for (auto face_iter = mesh->faces_begin(), face_end = mesh->faces_end(); face_iter != face_end; face_iter++)
    {
        const auto face = *face_iter;
        
        calculateSurface(*mesh, face, _Vd[face.idx()]);
    }
    
    TIMER_END(CalculateVd);
    
    constructQ();
    
    return true;
}

bool TransferSolver::setEmptySourceDeform(MeshPtr mesh)
{
    std::cout
    << "Source Deform" << std::endl
    << "\tVertices: " << mesh->n_vertices() << std::endl
    << "\tTriangles: " << mesh->n_faces() << std::endl;
    
    TIMER_START(CalculateVd);
    
    _Vd.resize(mesh->n_faces());
    
    for (auto face_iter = mesh->faces_begin(), face_end = mesh->faces_end(); face_iter != face_end; face_iter++)
    {
        const auto face = *face_iter;
        
        //calculateSurface(*mesh, face, _Vd[face.idx()]);
        _Vd[face.idx()] = Matrix3x3::Identity();
    }
    
    TIMER_END(CalculateVd);
    
    constructQ();
    
    return true;
}

bool TransferSolver::setTargetReference(MeshPtr mesh, CorrespondencePtr corr, bool buildVertexMap)
{
    std::cout
        << "Target Reference" << std::endl
        << "\tVertices: " << mesh->n_vertices() << std::endl
        << "\tTriangles: " << mesh->n_faces() << std::endl;
    
    _correspondence = corr;
    
    // Count number of correspondences.
    // Add at least 1 for each, for identity solution.
    _numCorrespondences = 0;
    
    for (int i = 0; i < mesh->n_faces(); i++)
    {
        const auto corr = _correspondence->get(i);
        
        _numCorrespondences += std::max(1, (int)corr.size());
    }
    
    if (!buildVertexMap)
    {
        _numVertices = mesh->n_vertices();
    }
    else
    {
        TIMER_START(BuildVertexMap);
        
        _vertexMap.resize(mesh->n_vertices(), INVALID);
        
        // OpenMesh will fix meshes, duplicating vertices.
        // Create a map to reduce the vertices to only unique vertices.
        int numDuplicates = 0;
        for (unsigned int i = 0; i < mesh->n_vertices(); i++)
        {
            if (_vertexMap[i] != INVALID)
                continue;
            
            for (int j = i + 1; j < mesh->n_vertices(); j++)
            {
                if (mesh->point(mesh->vertex_handle(i)) == mesh->point(mesh->vertex_handle(j)))
                {
                    _vertexMap[j] = i;
                    numDuplicates++;
                }
            }
        }
        
        _numVertices = mesh->n_vertices() - numDuplicates;
        
        if (numDuplicates > 0)
            std::cout << "\tUnique Vertices: " << _numVertices << std::endl;
        else
            std::cout << "\t(No Duplicate Vertices)" << std::endl;
        
        TIMER_END(BuildVertexMap);
    }
    
    TIMER_START(ConstructA);
    
    constructA(*mesh, _A);
    
    TIMER_END(ConstructA);
    
    TIMER_START(Compute);
    
    _At = _A.transpose();

    _AtA = _At * _A;

    _solver.compute(_AtA);
    
    TIMER_END(Compute);
    
    return checkSolverError();
}

bool TransferSolver::deform(MeshPtr targetDeform)
{
    TIMER_START(ConstructC);
    
    MatrixX c;
    constructC(*targetDeform, c);
    
    TIMER_END(ConstructC);
    
    TIMER_START(Solve);
    
    MatrixX AtC = _At * c;
    
    MatrixX x = _solver.solve(AtC);
    
    TIMER_END(Solve);
    
    if (!checkSolverError())
        return false;
    
    TIMER_START(CopyToMesh);
    
    copyTo(x, *targetDeform);
    
    TIMER_END(CopyToMesh);
    
    return true;
}

void TransferSolver::constructA(const Mesh& mesh, SparseMatrix& a)
{
    const auto rows = _numCorrespondences * 9;
    const auto cols = (_numVertices + mesh.n_faces()) * 3;
    
    std::cout
        << "Constructing A" << std::endl
        << "\tSize: " << rows << " x " << cols << std::endl;
    
    TripletList v;
    v.reserve(rows * 10);
    
    Matrix9x4 e;
    
    _row = 0;
    
    for(uint i = 0; i < mesh.n_faces(); i++)
    {
        const auto face = mesh.face_handle(i);
        
        const auto corr = _correspondence->get(i);
        const auto numCorr = std::max(1, (int)corr.size());
        
        constructE(mesh, face, e);
        
        for (int c = 0; c < numCorr; c++)
        {
            constructA(mesh, face, e, v);
        }
    }
    
    _A.resize(rows, cols);
    _A.setZero();
    
    _A.setFromTriplets(v.begin(), v.end());
    
    _A.makeCompressed();
}

void TransferSolver::constructA(const Mesh& mesh, const Mesh::FaceHandle& face, const Matrix9x4& e, TripletList& triplets)
{
    unsigned int vertexIdx[4];
    vertexIndices(mesh, face, vertexIdx);
    
    int j_row = 0;
    for (int coord = 0; coord < 3; coord++)
    {
        for (int eqn = 0; eqn < 3; eqn++, _row++, j_row++)
        {
            for (int vert = 0; vert < 4; vert++)
            {
                const int vertIdx = (int)vertexIdx[vert];
                
                triplets.push_back(Triplet{_row, vertIdx + coord, e(j_row, vert)});
            }
        }
    }
}

void TransferSolver::constructQ()
{
    if (_invVr.size() != _Vd.size())
        return;
    
    TIMER_START(ConstructQ);
    
    _Q.resize(_Vd.size());
    
    for (auto i = 0; i < _Vd.size(); i++)
    {
        //_Q[i] = _invVr[i] * _Vd[i];
        //_Q[i].transposeInPlace();
        
        calculateQ(_invVr[i], _Vd[i], _Q[i]);
    }
    
    TIMER_END(ConstructQ);
}

void TransferSolver::constructC(const Mesh& target, MatrixX& c)
{
    const auto rows = _numCorrespondences * 9;
    
    std::cout
        << "Constructing C" << std::endl
        << "\tSize: " << rows << " x " << 1 << std::endl;
    
    c.resize(rows, 1);
    c.setZero();
    
    _row = 0;
    
    for(auto i = 0; i < target.n_faces(); i++)
    {
        const auto& corr = _correspondence->get(i);
        
        if (corr.empty())
        {
            constructC(C_Identity, c);
        }
        else
        {
            for (auto j = 0; j < corr.size(); j++)
            {
                const auto corrFaceIdx = corr[j];
                
                constructC(_Q[corrFaceIdx], c);
            }
        }
    }
}

void TransferSolver::constructC(const Matrix9x1& q, MatrixX& c)
{
    //c.block<3, 3>(idx * 3, 0) = q;
    
//    c.block<3, 1>(idx * 9, 0) += q.row(0);
//    c.block<3, 1>((idx * 9) + 3, 0) += q.row(1);
//    c.block<3, 1>((idx * 9) + 6, 0) += q.row(2);
    
//    for (int i = 0; i < 3; i++)
//    {
//        for (int j = 0; j < 3; j++)
//        {
//            c(_row, 0) = q(i, j);
//            _row++;
//        }
//    }
    
    c.block<9, 1>(_row, 0) += q;
    _row += 9;
}

void TransferSolver::copyTo(const MatrixX& x, Mesh& mesh) const
{
    int logVerts = 3;
    
    std::cout << "Applying Transformation: " << std::endl;
    
    for (auto i = 0; i < mesh.n_vertices(); i++)
    {
        const auto vert = mesh.vertex_handle(i);
        const auto idx = vertexIndex(i);
        
        const auto p = mesh.point(vert);
        const auto np = OpenMesh::Vec3d(x(idx, 0), x(idx + 1, 0), x(idx + 2, 0));
        
        mesh.point(vert) = np;
        
        if (logVerts > 0)
        {
            std::cout << "\tVert " << vert.idx() << ": " << p << " -> " << np << std::endl;
            logVerts--;
        }
    }
}

unsigned int TransferSolver::vertexIndex(const Mesh::VertexHandle& vert) const
{
    return vertexIndex(vert.idx());
}

unsigned int TransferSolver::vertexIndex(unsigned int idx) const
{
    auto vertexIdx = _vertexMap.empty() ? idx : _vertexMap[idx];
    if (vertexIdx == INVALID)
        vertexIdx = idx;
    
    return vertexIdx * 3;
}

void TransferSolver::vertexIndices(const Mesh& mesh, const Mesh::FaceHandle& face, unsigned int vertices[]) const
{
    size_t i = 0;
    for (auto vertIter = mesh.cfv_begin(face), vertEnd = mesh.cfv_end(face); i < 3 && vertIter != vertEnd; vertIter++, i++)
    {
        vertices[i] = vertexIndex((*vertIter).idx());
    }
    
    vertices[3] = (unsigned int)(_numVertices + face.idx()) * 3;
}
