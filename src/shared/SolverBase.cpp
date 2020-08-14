//
//  SolverBase.cpp
//  Deform
//
//  Created by Kyle on 12/7/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "SolverBase.h"

#include "Util.h"

#include <iostream>

Matrix9x1 SolverBase::C_Identity;

SolverBase::SolverBase()
{
    Eigen::initParallel();
    
    C_Identity <<
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
}

bool SolverBase::checkSolverError() const
{
    if (_solver.info() == Eigen::Success)
        return true;
    
    std::cerr
    << "*******" << std::endl
    << "**Eigen Solver Error: " << Error(_solver.info()) << std::endl
    << "*******" << std::endl;
    
    return false;
}

void SolverBase::constructE(const Mesh& mesh, const Mesh::FaceHandle& face, Matrix9x4& e) const
{
    Matrix3x3 invS;
    calculateInvSurface(mesh, face, invS);
    
    constructE(invS, e);
}

void SolverBase::constructE(const Matrix3x3& invS, Matrix9x4& e) const
{
    /***
     invS
     ---
     m00 m01 m02
     m10 m11 m12
     m20 m21 m22
     
     e
     ---
     v1                 v2  v3  v4/p
     -(m00 + m10 + m20) m00 m10 m20
     -(m01 + m11 + m21) m01 m11 m21
     -(m02 + m12 + m22) m02 m12 m22
     
     -(m00 + m10 + m20) m00 m10 m20
     -(m01 + m11 + m21) m01 m11 m21
     -(m02 + m12 + m22) m02 m12 m22
     
     -(m00 + m10 + m20) m00 m10 m20
     -(m01 + m11 + m21) m01 m11 m21
     -(m02 + m12 + m22) m02 m12 m22
     ***/
    
    for (int i = 0, row = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++, row++)
        {
            e(row, 0) = -(invS(0, j) + invS(1, j) + invS(2, j));
            e(row, 1) = invS(0, j);
            e(row, 2) = invS(1, j);
            e(row, 3) = invS(2, j);
        }
    }
}

void SolverBase::calculateQ(const Mesh& ref, const Mesh::FaceHandle& refFace, const Mesh& def, const Mesh::FaceHandle& defFace, Matrix3x3& q) const
{
    Matrix3x3 invVr;
    calculateInvSurface(ref, refFace, invVr);
    
    Matrix3x3 Vd;
    calculateSurface(def, defFace, Vd);
    
    calculateQ(invVr, Vd, q);
}

void SolverBase::calculateQ(const Matrix3x3& invVr, const Matrix3x3& vd, Matrix9x1& q) const
{
    Matrix3x3 q33;
    calculateQ(invVr, vd, q33);
    
    for (int i = 0, r = 0; i < 3; i++)
        for (int j = 0; j < 3; j++, r++)
            q(r, 0) = q33(i, j);
}

void SolverBase::calculateQ(const Matrix3x3& invVr, const Matrix3x3& vd, Matrix3x3& q) const
{
    //q = invVr * vd;
    q = vd * invVr;
    //q.transposeInPlace();
    //q = Matrix3x3::Identity();
}

void SolverBase::calculateSurface(const Mesh& ref, const Mesh::FaceHandle& refFace, Matrix3x3& s) const
{
    constructTriangleNormMatrix(ref, refFace, s);
}

void SolverBase::calculateInvSurface(const Mesh& ref, const Mesh::FaceHandle& refFace, Matrix3x3& inv) const
{
    Matrix3x3 Vr;
    calculateSurface(ref, refFace, Vr);
    
    inv = Vr.inverse();
}

void SolverBase::constructTriangleNormMatrix(const Mesh& mesh, const Mesh::FaceHandle& face, Matrix3x3& v) const
{
    Mesh::VertexHandle vertices[3];
    
    FaceVertices(mesh, face, vertices);
    
    const auto v0 = mesh.point(vertices[0]);
    const auto e0 = toEigen(mesh.point(vertices[1]) - v0);
    const auto e1 = toEigen(mesh.point(vertices[2]) - v0);
    const auto n = calculatePhantom(toEigen(v0), e0, e1);
    
    v.col(0) = e0;
    v.col(1) = e1;
    v.col(2) = n;
}

Eigen::Vector3d SolverBase::calculatePhantom(const Eigen::Vector3d& v0, const Eigen::Vector3d& e0, const Eigen::Vector3d& e1) const
{
    const auto n = e0.cross(e1);
    const auto l = sqrt(n.norm());
    
    //return v0 + (n / l);
    return (n / l);
}
