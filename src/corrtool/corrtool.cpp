//
//  corrtool.cpp
//  CorrTool
//
//  Created by Kyle on 12/19/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "MeshView.h"
#include "CorrespondenceManager.h"

#include "gl.h"

#include <stdio.h>

#include <memory>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 4)
    {
        std::cerr << "Too few arguments" << std::endl;
        exit(1);
    }
    
    std::string modelA = argv[1];
    std::string modelB = argv[2];
    std::string outputPath = argv[3];
    
    glutInit(&argc, argv);
    
    auto corrManager = std::make_shared<CorrespondenceManager>();
    
    corrManager->read(outputPath);
    
    auto meshViewA = MakeMeshView();
    if (!meshViewA->loadMesh(modelA))
    {
        std::cerr << "Failed to load model: " << modelA << std::endl;
        exit(1);
    }
    
    auto meshViewB = MakeMeshView();
    if (!meshViewB->loadMesh(modelB))
    {
        std::cerr << "Failed to load model: " << modelB << std::endl;
    }
    
    meshViewA->create("A", 50, 50, 640, 480);
    meshViewA->setCorrespondenceManager(corrManager, CorrespondenceManager::TARGET_A);
    
    meshViewB->create("B", 700, 50, 640, 480);
    meshViewB->setCorrespondenceManager(corrManager, CorrespondenceManager::TARGET_B);
    
    glutMainLoop();
}
