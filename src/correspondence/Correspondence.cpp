// Correspondence.cpp : Defines the entry point for the console application.
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
    cxxopts::Options options("correspondence", "Generate face-to-face correspondence for two meshes from sparse vertex correspondence");

    options.add_options()
            ("s,source-ref", "Path to the source reference mesh", cxxopts::value<std::string>())
            ("t,target-ref", "Path to the target reference mesh", cxxopts::value<std::string>())
            ("v,vertex-corr", "Path to the vertex correspondence file", cxxopts::value<std::string>())
            ("i,intermediate", "Path to the intermediate directory", cxxopts::value<std::string>(), "(Optional)")
            ("o,output", "Path to save the deformed target mesh to", cxxopts::value<std::string>())
            ;

    std::string sourceRefPath;
    std::string targetRefPath;
    std::string vertCorrespondencePath;
    std::string outputPath;
    std::string intermediatePath;

    try
    {
        auto result = options.parse(argc, argv);

        if (!result.count("s") || !result.count("t") || !result.count("v") || !result.count("o"))
        {
            std::cout << options.help() << std::endl;
            exit(1);
        }

        sourceRefPath = result["source-ref"].as<std::string>();
        targetRefPath = result["target-ref"].as<std::string>();
        vertCorrespondencePath = result["vertex-corr"].as<std::string>();
        outputPath = result["output"].as<std::string>();

        if (result.count("i")) {
            intermediatePath = result["intermediate"].as<std::string>();
        }
    }
    catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    MeshPtr sourceRef = ReadMesh(sourceRefPath, true);
    MeshPtr targetRef = ReadMesh(targetRefPath, true);

    CorrespondencePtr vertCorrespondence = std::make_shared<SparseCorrespondence>();
    if (!vertCorrespondence->read(vertCorrespondencePath))
    {
        std::cerr << "Failed to correspondence mesh at [" << vertCorrespondencePath << "]" << std::endl;

        exit(1);
    }

    auto anchorMap = CorrespondenceUtil::BuildConstraints(vertCorrespondence, targetRef);

    std::cout << std::endl << "=Correspondence Resolver=" << std::endl;

    TIMER_START(CorrespondenceResolver)

    CorrespondenceSolver resolver;
    resolver.setSourceReference(sourceRef);
    resolver.setTargetReference(targetRef);
    resolver.setVertexConstraints(anchorMap);

    if (!intermediatePath.empty())
    {


        resolver.setStepCallback([&intermediatePath](int step, MeshPtr mesh) {
            std::stringstream path;
            path << intermediatePath << "/correspondence-step-" << step << ".obj";

            WriteMesh(path.str(), mesh);
        });
    }

    resolver.resolve();

    resolver.faceCorrespondence().write(outputPath);

    TIMER_END(CorrespondenceResolver)

    std::cout << "Complete" <<std::endl;

    return 0;
}

