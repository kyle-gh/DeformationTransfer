// Transfer.cpp : Defines the entry point for the console application.
//

#include "../shared/Mesh.h"

#include "../shared/transfer/TransferSolver.h"
#include "../shared/correspondence/CorrespondenceSolver.h"
#include "../shared/correspondence/CorrespondenceUtil.h"

#include "../shared/SparseCorrespondence.h"

#include "../shared/Timing.h"

#include <iostream>
#include <memory>
#include <fstream>
#include <iomanip>
#include <filesystem>

int main(int argc, char* argv[])
{
    if (argc < 4)
    {
        std::cerr << "Too few arguments" << std::endl;
        exit(1);
    }

    std::string sourceRefPath = argv[1];
    std::string sourceDeformPath = argv[2];
    std::string targetRefPath = argv[3];
    std::string vertCorrespondencePath = argv[4];
    std::string outputPath = argv[5];
    std::string faceCorrespondencePath = std::tmpnam(nullptr);

    MeshPtr sourceRef = ReadMesh(sourceRefPath, true);
    MeshPtr sourceDeform = ReadMesh(sourceDeformPath, true);
    
    MeshPtr targetRef = ReadMesh(targetRefPath, true);
    
    MeshPtr targetDeform = MakeMesh(targetRef);
    
    CorrespondenceSolver::ConstraintMapPtr anchorMap = nullptr;

    CorrespondencePtr vertCorrespondence = std::make_shared<SparseCorrespondence>();
    vertCorrespondence->read(vertCorrespondencePath);

    anchorMap = CorrespondenceUtil::BuildConstraints(vertCorrespondence, targetRef);

    std::cout << std::endl << "=Correspondence Resolver=" << std::endl;
    
    TIMER_START(CorrespondenceResolver)

    CorrespondenceSolver resolver;
    resolver.setSourceReference(sourceRef);
    resolver.setTargetReference(targetRef);
    resolver.setVertexConstraints(anchorMap);
    
//    resolver.setStepCallback([](int step, MeshPtr m) {
//        char path[1024];
//        sprintf(path, "results/template-random1-%d.obj", step);
//        std::string p = path;
//        WriteMesh(p, m);
//    });

    resolver.resolve();

    resolver.faceCorrespondence().write(faceCorrespondencePath);

    TIMER_END(CorrespondenceResolver)

    std::cout << std::endl << "=Deformation Transfer=" << std::endl;
    
    TIMER_START(Transfer);
    
    auto faceCorrespondence = std::make_shared<DenseCorrespondence>();
    faceCorrespondence->read(faceCorrespondencePath);

    std::remove(faceCorrespondencePath.c_str());

    std::stringstream path;
    
    TransferSolver xfer;
    
    xfer.setSourceReference(sourceRef);
    
    if (!xfer.setTargetReference(targetRef, faceCorrespondence))
    {
        std::cerr << "Failed to set target reference" << std::endl;
        exit(1);
    }
    
    xfer.setSourceDeform(sourceDeform);

    auto success = xfer.deform(targetDeform);

    TIMER_END(Transfer);

    if (!success)
    {
        std::cerr << "Failed to transfer deformation" << std::endl;
        return 1;
    }

    WriteMesh(outputPath, targetDeform);

    std::cout << "Complete" <<std::endl;
    
    return 0;
}

