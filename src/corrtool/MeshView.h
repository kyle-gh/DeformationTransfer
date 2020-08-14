//
//  MeshView.hpp
//  CorrTool
//
//  Created by Kyle on 12/19/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef MeshView_hpp
#define MeshView_hpp

#include "../shared/Mesh.h"

#include "CorrespondenceManager.h"

#include <stdio.h>

#include <map>
#include <vector>
#include <memory>

class MeshView : public std::enable_shared_from_this<MeshView>
{
public:
    static std::shared_ptr<MeshView> getViewer(int window) { return _Viewers[window]; }
    static std::map<int, std::shared_ptr<MeshView>>& getViewers() { return _Viewers; }
    
    MeshView();
    ~MeshView();
    
    bool create(const std::string& title, int x, int y, int w, int h);
    
    bool loadMesh(const std::string& path);
    
    void setCorrespondenceManager(CorrespondenceManagerPtr m, CorrespondenceManager::Target target);
    
    void idle();
    void display();
    void keyboard(int key, int x, int y);
    void mouse(int button, int state, int x, int y);
    void reshape(int w, int h);
    
    void postRedisplay();
    
private:
    typedef OpenMesh::Vec3d Vec3;
    
    static std::map<int, std::shared_ptr<MeshView>> _Viewers;
    
    std::string _title;
    
    int _windowId;
    
    CorrespondenceManagerPtr _manager;
    CorrespondenceManager::Target _target;
    
    Vec3 _viewPosition;
    Vec3 _viewCenter;
    Vec3 _viewDir;
    double _viewDistance;
    
    double _viewDistanceMin;
    double _viewDistanceMax;
    
    double _rotationVertical;
    double _rotationHorizontal;
    
    double _fov;
    double _near;
    double _far;
    OpenMesh::Vec2i _viewport;
    
    MeshPtr _mesh;
    
    Vec3 _boundsMin;
    Vec3 _boundsMax;
    Vec3 _boundsCenter;
    
    std::vector<unsigned int> _pointIndices;
    std::vector<unsigned int> _faceIndices;
    
    int _selected;
    
    double _vertexSize;
    
    void pick(int x, int y);
    
    void buildIndices();
    
    void slide(double distance);
    void rotateAround(double horizontal, double vertical);
    
    void setView(const Vec3& dir, double distance);
};

typedef std::shared_ptr<MeshView> MeshViewPtr;

inline MeshViewPtr MakeMeshView() { return std::make_shared<MeshView>(); }

#endif /* MeshView_hpp */
