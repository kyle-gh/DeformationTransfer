// Test.cpp : Defines the entry point for the console application.
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
    std::string dataPath = argv[1];
    std::string outputPath = argv[2];

    const std::string vertCorrespondencePath = dataPath + "/horse_camel_vert.corr";
    const std::string faceCorrespondencePath = outputPath + "/horse_camel_face.corr";

    const auto numPoses = 10;
    const std::string horseDir = dataPath  + "/horse/";
    const std::string horseRefPath = horseDir + "horse-reference.obj";
    const std::string camelDir = dataPath  + "/camel/";
    const std::string camelRefPath = camelDir + "camel-reference.obj";

    std::stringstream path;

    MeshPtr sourceRef = ReadMesh(horseRefPath, true);
    MeshPtr targetRef = ReadMesh(camelRefPath, true);
    MeshPtr targetDeform = MakeMesh(targetRef);

    MeshPtr mergedMesh = MakeMesh();

    CorrespondenceSolver::ConstraintMapPtr anchorMap = nullptr;

    CorrespondencePtr vertCorrespondence = std::make_shared<SparseCorrespondence>();
    if (!vertCorrespondence->read(vertCorrespondencePath))
    {
        std::cerr << "Failed to read Vertex Correspondence \"" << vertCorrespondencePath << "\"" << std::endl;
        exit(1);
    }

    anchorMap = CorrespondenceUtil::BuildConstraints(vertCorrespondence, targetRef, false);

    std::cout << std::endl << "=Correspondence Resolver=" << std::endl;

    TIMER_START(CorrespondenceResolver)

    CorrespondenceSolver resolver;
    resolver.setSourceReference(sourceRef);
    resolver.setTargetReference(targetRef);
    resolver.setVertexConstraints(anchorMap);

    resolver.setStepCallback([&outputPath](int step, MeshPtr mesh) {
        std::stringstream path;
        path << outputPath << "/correspondence-step-" << std::setfill('0') << std::setw(2) << step << ".obj";
        WriteMesh(path.str(), mesh);
    });

    resolver.resolve();

    resolver.faceCorrespondence().write(faceCorrespondencePath);

    TIMER_END(CorrespondenceResolver)

    std::cout << std::endl << "=Deformation Transfer=" << std::endl;

    TIMER_START(Transfer);

    auto faceCorrespondence = std::make_shared<DenseCorrespondence>();
    faceCorrespondence->read(faceCorrespondencePath);

    TransferSolver xfer;

    xfer.setSourceReference(sourceRef);

    if (!xfer.setTargetReference(targetRef, faceCorrespondence, true))
    {
        std::cerr << "Failed to set target reference" << std::endl;
        exit(1);
    }

    for (int pose = 1; pose <= numPoses; pose++)
    {
        path.str("");
        path << horseDir << "horse-" << std::setfill('0') << std::setw(2) << pose << ".obj";

        std::cout << "Deforming to " << path.str() << std::endl;

        auto sourceDeform = ReadMesh(path.str());

        xfer.setSourceDeform(sourceDeform);

        auto success = xfer.deform(targetDeform);

        TIMER_END(Transfer);

        if (!success)
        {
            std::cerr << "Failed to transfer deformation" << std::endl;
            return 1;
        }

        path.str("");
        path << outputPath << "/camel-" << std::setfill('0') << std::setw(2) << pose << "-deform.obj";

        WriteMesh(path.str(), targetDeform);
    }

    std::cout << "Complete" << std::endl;

    return 0;
}

