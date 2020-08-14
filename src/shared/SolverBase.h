//
//  SolverBase.h
//  Deform
//
//  Created by Kyle on 12/7/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef SolverBase_h
#define SolverBase_h

#include "Mesh.h"

#include "Matrix.h"

class SolverBase
{
protected:
    static Matrix9x1 C_Identity;
    
    SolverBase();
    
    Eigen::SimplicialLDLT<SparseMatrix> _solver;
    
    bool checkSolverError() const;
    
    void constructE(const Mesh& mesh, const Mesh::FaceHandle& face, Matrix9x4& e) const;
    void constructE(const Matrix3x3& invS, Matrix9x4& e) const;
    
    void calculateQ(const Mesh& ref, const Mesh::FaceHandle& refFace, const Mesh& def, const Mesh::FaceHandle& defFace, Matrix3x3& q) const;
    void calculateQ(const Matrix3x3& invVr, const Matrix3x3& vd, Matrix9x1& q) const;
    void calculateQ(const Matrix3x3& invVr, const Matrix3x3& vd, Matrix3x3& q) const;
    
    void calculateSurface(const Mesh& ref, const Mesh::FaceHandle& refFace, Matrix3x3& s) const;
    void calculateInvSurface(const Mesh& ref, const Mesh::FaceHandle& refFace, Matrix3x3& inv) const;
    
    void constructTriangleNormMatrix(const Mesh& mesh, const Mesh::FaceHandle& face, Matrix3x3& v) const;
    
    Eigen::Vector3d calculatePhantom(const Eigen::Vector3d& v0, const Eigen::Vector3d& e0, const Eigen::Vector3d& e1) const;
};

#endif /* SolverBase_h */
