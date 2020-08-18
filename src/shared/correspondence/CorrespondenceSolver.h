//
//  CorrespondenceResolver.h
//  Deform
//
//  Created by Kyle on 12/7/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef CorrespondenceSolver_h
#define CorrespondenceSolver_h

#include "../SolverBase.h"

#include "../Grid.h"

#include "../DenseCorrespondence.h"
#include "../SingleDenseCorrespondence.h"

#include <functional>

class CorrespondenceSolver : public SolverBase
{
public:
    struct Weights
    {
        double smooth;
        double identity;
        double closest;
    };
    
    typedef std::map<size_t, Mesh::Point> ConstraintMap;
    typedef std::shared_ptr<ConstraintMap> ConstraintMapPtr;
    
    typedef std::function<void(int, MeshPtr)> StepCallback;
    
    CorrespondenceSolver();
    ~CorrespondenceSolver();
    
    bool setSourceReference(MeshPtr mesh);
    bool setTargetReference(MeshPtr mesh);
    
    bool setVertexConstraints(ConstraintMapPtr map);
    
    void setWeights(const std::vector<Weights>& weights);
    
    void setStepCallback(StepCallback callback);
    
    const Correspondence& faceCorrespondence() const;
    
    bool resolve();
    
private:
    std::vector<Weights> _weights;
    
    StepCallback _stepCallback;
    
    int _maxCorrespondence;
    
    MeshPtr _source;
    MeshPtr _target;
    
    ConstraintMapPtr _anchorMap;
    
    std::vector<unsigned int> _vertexMap;
    int _freeVertices;
    
    DenseCorrespondence _faceCorr;
    
    DenseCorrespondence _faceAdjacency;
    
    Grid _grid;
    
    SingleDenseCorrespondence _nearestCorr;
    
    std::vector<Matrix3x3> _invSurface;
    
    std::vector<Matrix9x4> _es;
    std::vector<Matrix9x1> _cs;
    
    int _row;
    
    TripletList _m;
    SparseMatrix _A;
    
    MatrixX _c;
    MatrixX _x;
    
    void resetSolve();
    void solveSI(const Weights& weights);
    void solveSIC(const Weights& weights);
    void constructCorrespondence();
    
    void appendSmoothness(double weight, TripletList& m, MatrixX& c);
    void appendSmoothness(const Mesh::FaceHandle& face, double weight, TripletList& m, MatrixX& c);
    
    void appendIdentity(double weight, TripletList& m, MatrixX& c);
    void appendIdentity(const Mesh::FaceHandle& face, double weight, TripletList& m, MatrixX& c);
    
    void appendClosest(const Correspondence& nearest, double weight, TripletList& m, MatrixX& c);
    void appendClosest(const Mesh::VertexHandle& vert, const Correspondence& nearest, double weight, TripletList& m, MatrixX& c);
    
    void appendEC(const Mesh::FaceHandle& face, double weight, TripletList& m, MatrixX& c);
    void appendEC(const Mesh::FaceHandle& face, const Matrix9x4& face_e, const Matrix9x1& face_c, double weight, TripletList& m, MatrixX& c);
    
    void constructECs(const Mesh& source, const Mesh& target);
    void constructEC(const Mesh& source, const Mesh& target, const Mesh::FaceHandle& face, Matrix9x4& e, Matrix9x1& c);
    
    void constructInvSurfaces(const Mesh& mesh, std::vector<Matrix3x3>& invSurfaces);
    
    void vertexIndices(const Mesh& mesh, const Mesh::FaceHandle& face, unsigned int indices[]);
    unsigned int vertexIndex(const Mesh::VertexHandle& vert) const;
    unsigned int vertexIndex(unsigned int vertIdx) const;
    
    bool solve(SparseMatrix& a, MatrixX& c, MatrixX& x);
    
    void copyTo(const MatrixX& x, Mesh& mesh);
};

#endif /* CorrespondenceSolver_h */
