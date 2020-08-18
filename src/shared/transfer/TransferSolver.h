//
//  TransferSolver.h
//  Deform
//
//  Created by Kyle on 11/18/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef TransferSolver_h
#define TransferSolver_h

#include "SolverBase.h"

#include "../Correspondence.h"

class TransferSolver : public SolverBase
{
public:
    TransferSolver();
    ~TransferSolver();
    
    bool setSourceReference(MeshPtr mesh);
    bool setEmptySourceReference(MeshPtr mesh);
    bool setSourceDeform(MeshPtr mesh);
    bool setEmptySourceDeform(MeshPtr mesh);
    
    bool setTargetReference(MeshPtr mesh, CorrespondencePtr corr, bool buildVertexMap = false);
    
    bool setCorrespondence(CorrespondencePtr corr);
    
    bool deform(MeshPtr targetDeform);
    
private:
    CorrespondencePtr _correspondence;
    
    size_t _numCorrespondences;
    
    std::vector<unsigned int> _vertexMap;
    
    size_t _numVertices;
    
    int _row;
    
    std::vector<Matrix3x3> _invVr;
    std::vector<Matrix3x3> _Vd;
    std::vector<Matrix9x1> _Q;
    
    SparseMatrix _A;
    SparseMatrix _At;
    SparseMatrix _AtA;
    
    void constructA(const Mesh& mesh, SparseMatrix& a);
    void constructA(const Mesh& mesh, const Mesh::FaceHandle& face, const Matrix9x4& e, TripletList& triplets);
    
    void constructQ();
    
    void constructC(const Mesh& target, MatrixX& c);
    void constructC(const Matrix9x1& q, MatrixX& c);
    
    void copyTo(const MatrixX& x, Mesh& mesh) const;
    
    unsigned int vertexIndex(const Mesh::VertexHandle& vert) const;
    unsigned int vertexIndex(unsigned int idx) const;
    void vertexIndices(const Mesh& mesh, const Mesh::FaceHandle& face, unsigned int vertices[]) const;
};

#endif /* TransferSolver_h */
