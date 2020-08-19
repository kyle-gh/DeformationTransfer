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

#include <cxxopts.hpp>

int main(int argc, char* argv[])
{
    cxxopts::Options options("transfer", "Transfer deformation from one mesh to another");

    options.add_options()
        ("r,source-ref", "Path to the source reference mesh", cxxopts::value<std::string>())
        ("d,source-deform", "Path to the source deform mesh", cxxopts::value<std::string>())
        ("t,target-ref", "Path to the target reference mesh", cxxopts::value<std::string>())
        ("v,vertex-corr", "Path to the vertex correspondence file", cxxopts::value<std::string>(), "Vertex or face correspondence must be provided")
        ("f,face-corr", "Path to the face correspondence file", cxxopts::value<std::string>(), "Vertex or face correspondence must be provided")
        ("o,output", "Path to save the deformed target mesh to", cxxopts::value<std::string>())
        ;

    std::string sourceRefPath;
    std::string sourceDeformPath;
    std::string targetRefPath;
    std::string vertCorrespondencePath;
    std::string faceCorrespondencePath;
    std::string outputPath;

    try
    {
        auto result = options.parse(argc, argv);

        if (!result.count("r") || !result.count("d") || !result.count("t") || !result.count("o"))
        {
            std::cout << options.help() << std::endl;
            exit(1);
        }

        if (!result.count("v") && !result.count("f"))
        {
            std::cout << options.help() << std::endl;
            exit(1);
        }

        sourceRefPath = result["source-ref"].as<std::string>();
        sourceDeformPath = result["source-deform"].as<std::string>();
        targetRefPath = result["target-ref"].as<std::string>();

        if (result.count("v"))
        {
            vertCorrespondencePath = result["vertex-corr"].as<std::string>();
        }
        else
        {
            faceCorrespondencePath = result["face-corr"].as<std::string>();
        }

        outputPath = result["output"].as<std::string>();
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    MeshPtr sourceRef = ReadMesh(sourceRefPath, true);
    MeshPtr sourceDeform = ReadMesh(sourceDeformPath, true);

    MeshPtr targetRef = ReadMesh(targetRefPath, true);

    MeshPtr targetDeform = MakeMesh(targetRef);

    auto tempFaceCorrPath = false;
    if (faceCorrespondencePath.empty())
    {
        faceCorrespondencePath = std::tmpnam(nullptr);
        tempFaceCorrPath = true;
    }

    if (!vertCorrespondencePath.empty())
    {
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
    }

    std::cout << std::endl << "=Deformation Transfer=" << std::endl;
    
    TIMER_START(Transfer);
    
    auto faceCorrespondence = std::make_shared<DenseCorrespondence>();
    faceCorrespondence->read(faceCorrespondencePath);

    if (tempFaceCorrPath)
    {
        std::remove(faceCorrespondencePath.c_str());
    }

    std::stringstream path;
    
    TransferSolver xfer;
    
    xfer.setSourceReference(sourceRef);
    
    if (!xfer.setTargetReference(targetRef, faceCorrespondence, true))
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

