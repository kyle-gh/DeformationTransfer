//
//  MeshView.cpp
//  CorrTool
//
//  Created by Kyle on 12/19/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "MeshView.h"

#include "../shared/Matrix.h"

#include "gl.h"

#define DEG2RAD (M_PI / 180.0)

//#define SPHERE_RADIUS 2
//0.01

#define SPHERE_RATIO (2.0/250.0)

inline double clamp(double v, double min, double max)
{
    if (min > v)
        return min;
    else if (max < v)
        return max;
    return v;
}

namespace MeshViewInterface
{
    void postRedisplay()
    {
        auto& viewers = MeshView::getViewers();
        
        for (auto iter = viewers.begin(); iter != viewers.end(); iter++)
            iter->second->postRedisplay();
    }
    
    void idle()
    {
        MeshView::getViewer(glutGetWindow())->idle();
    }
    
    void display()
    {
        MeshView::getViewer(glutGetWindow())->display();
    }
    
    void keyboard(unsigned char key, int x, int y)
    {
        if ((glutGetModifiers() & GLUT_ACTIVE_SHIFT) != 0)
            MeshView::getViewer(glutGetWindow())->keyboard(key, x, y);
        else
        {
            auto& viewers = MeshView::getViewers();
            
            for (auto iter = viewers.begin(); iter != viewers.end(); iter++)
                iter->second->keyboard(std::tolower(key), x, y);
            
            postRedisplay();
        }
    }
    
    void mouse(int button, int state, int x, int y)
    {
        //if ((glutGetModifiers() & GLUT_ACTIVE_CTRL) == 0)
        //    return;
        
        MeshView::getViewer(glutGetWindow())->mouse(button, state, x, y);
    }
    
    void reshape(int w, int h)
    {
        MeshView::getViewer(glutGetWindow())->reshape(w, h);
    }
}

std::map<int, std::shared_ptr<MeshView>> MeshView::_Viewers;

MeshView::MeshView()
{
    _windowId = -1;
    
    _fov = 45.0f;
    _near = 0.1;
    _far = 1000.0;
    
    _rotationHorizontal = 180;
    _rotationVertical = 0;
}

MeshView::~MeshView()
{
    if (_windowId != -1)
        glutDestroyWindow(_windowId);
}

void MeshView::setCorrespondenceManager(CorrespondenceManagerPtr m, CorrespondenceManager::Target target)
{
    _manager = m;
    _target = target;
}

bool MeshView::create(const std::string& title, int x, int y, int w, int h)
{
    _title = title;
    
    _viewport[0] = w;
    _viewport[1] = h;
    
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_ALPHA);
    glutInitWindowSize(w, h);
    glutInitWindowPosition(x, y);
    
    _windowId = glutCreateWindow(_title.c_str());
    _Viewers[_windowId] = shared_from_this();
    
    glutDisplayFunc(MeshViewInterface::display);
    glutKeyboardFunc(MeshViewInterface::keyboard);
    glutMouseFunc(MeshViewInterface::mouse);
    glutReshapeFunc(MeshViewInterface::reshape);
    
    
    
    glEnable(GL_DEPTH_TEST);
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    GLfloat position[] = {0, 1000, -1000};
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    
    glShadeModel(GL_SMOOTH);
    
    return true;
}

bool MeshView::loadMesh(const std::string& path)
{
    _mesh = ReadMesh(path);
    if (_mesh == nullptr)
        return false;
    
    buildIndices();
    
    Bounds(*_mesh, _boundsMin, _boundsMax);
    _boundsCenter = ((_boundsMax - _boundsMin) / 2) + _boundsMin;
    
    _viewCenter = _boundsCenter;
    
    auto maxDistance = (_boundsMax - _viewCenter).norm();
    
    _viewDistanceMin = 0.1f;
    _viewDistanceMax = 10 * maxDistance;
    
    setView(Vec3(0, 0, -1), maxDistance * 2);
    
    return true;
}

void MeshView::buildIndices()
{
    _faceIndices.resize(_mesh->n_faces() * 3);
    
    auto offset = 0;
    
    for (auto face_iter = _mesh->faces_begin(), face_end = _mesh->faces_end(); face_iter != face_end; face_iter++)
    {
        const auto face = *face_iter;

        for (auto vert_iter = _mesh->fv_begin(face), vert_end = _mesh->fv_end(face); vert_iter != vert_end; vert_iter++)
        {
            _faceIndices[offset] = (*vert_iter).idx();
            offset++;
        }
    }
    
    _pointIndices.resize(_mesh->n_vertices());
    
    for (auto i = 0; i < _mesh->n_vertices(); i++)
    {
        _pointIndices[i] = i;
    }
}

void MeshView::rotateAround(double horizontal, double vertical)
{
    _rotationVertical = clamp(_rotationVertical + vertical, -89.99, 89.99);
    _rotationHorizontal += horizontal;
    
    const auto sh = std::sin(_rotationHorizontal * DEG2RAD);
    const auto ch = std::cos(_rotationHorizontal * DEG2RAD);
    
    auto viewDir = Vec3(sh, _rotationVertical / 90, ch);
    viewDir.normalize();
    
    setView(viewDir, _viewDistance);
}

void MeshView::slide(double distance)
{
    setView(_viewDir, _viewDistance + distance);
}

void MeshView::setView(const Vec3& dir, double distance)
{
    _viewDir = dir;
    _viewDistance = clamp(distance, _viewDistanceMin, _viewDistanceMax);
    
    _viewPosition = _viewCenter + (_viewDir * _viewDistance);
    
    _vertexSize = SPHERE_RATIO * _viewDistance;
}

void MeshView::idle()
{
    // glutPostRedisplay();
}

void MeshView::display()
{
    if (_mesh == nullptr)
        return;
    
    glViewport(0, 0, _viewport[0], _viewport[1]);
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    
    glLoadIdentity();
    
    gluPerspective(_fov, (GLfloat)_viewport[0]/(GLfloat)_viewport[1], _near, _far);
    
    glMatrixMode(GL_MODELVIEW);
    
    glLoadIdentity();

    gluLookAt(_viewPosition[0], _viewPosition[1], _viewPosition[2],
              _viewCenter[0], _viewCenter[1], _viewCenter[2],
              0, 1, 0);
    
    // Draw Faces/Edges

    glEnable(GL_LIGHTING);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    glColor3f(1.0, 1.0, 1.0);

    glVertexPointer(3, GL_DOUBLE, 0, _mesh->points());
    glNormalPointer(GL_DOUBLE, 0, _mesh->vertex_normals());

    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(_faceIndices.size()), GL_UNSIGNED_INT, &_faceIndices[0]);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    
    // Draw Vertices
    
    glDisable(GL_LIGHTING);
    
    for (auto i = 0; i < _mesh->n_vertices(); i++)
    {
        const auto p = _mesh->points()[i];

        if (_manager->isActive(_target, i))
            glColor3f(0.0, 1.0, 0.0);
        else if (_manager->hasCorrespondence(_target, i))
            glColor3f(0.0, 0.0, 1.0);
        else
            continue;//glColor3f(0.4, 0.4, 0.4);
        
        glPushMatrix();

        glTranslated(p[0], p[1], p[2]);
        glutSolidSphere(_vertexSize, 5, 5);

        glPopMatrix();
    }
    
    glutSwapBuffers();
}

void MeshView::pick(int x, int y)
{
    Vec3 m_start;
    Vec3 m_end;
    
    double matModelView[16];
    double matProjection[16];
    int viewport[4];
    
    glGetDoublev(GL_MODELVIEW_MATRIX, matModelView);
    glGetDoublev(GL_PROJECTION_MATRIX, matProjection);
    glGetIntegerv(GL_VIEWPORT, viewport);
    
    double winX = (double)x;
    double winY = viewport[3] - (double)y;
    
    gluUnProject(winX, winY, 0.0, matModelView, matProjection,
                 viewport, &m_start[0], &m_start[1], &m_start[2]);
    gluUnProject(winX, winY, 1.0, matModelView, matProjection,
                 viewport, &m_end[0], &m_end[1], &m_end[2]);
    
    std::cout
        << "Pick @ (" << winX << ", " << winY << ")" << std::endl
        << "\t[" << m_start << "] -> [" << m_end << "]" << std::endl;
    
    const auto u = m_end - m_start;
    const auto uNorm = u.norm();
    
    auto minDistance = __DBL_MAX__;
    auto selected = -1;
    
    for (auto i = 0; i < _mesh->n_vertices(); i++)
    {
        const auto& p = _mesh->points()[i];
        const auto& n = _mesh->vertex_normals()[i];
        
        // Check vertices that are facing towards the camera
        if ((u|n) > 0)
            continue;
        
        const auto d = ((p - m_start) % u).norm() / uNorm;
        
        if (d < minDistance)
        {
            selected = i;
            minDistance = d;
        }
    }
    
    if (selected != -1)
    {
        std::cout
            << "\tSelected: " << selected << std::endl
            << "\t\t[" << _mesh->points()[selected] << "] :: " << minDistance << " (" << _vertexSize << ")" << std::endl;
        
        _manager->setActive(_target, selected);
    }
}

void MeshView::keyboard(int key, int x, int y)
{
    const auto rotationSpeed = 2.0;
    const auto translationSpeed = 0.5;
    
    switch(key)
    {
        case 27:
            exit(1);
            
        case 'a':
            rotateAround(-rotationSpeed, 0);
            break;
            
        case 's':
            rotateAround(0, -rotationSpeed);
            break;
            
        case 'w':
            rotateAround(0, rotationSpeed);
            break;
            
        case 'd':
            rotateAround(rotationSpeed, 0);
            break;
            
        case 'q':
            slide(translationSpeed);
            break;
            
        case 'e':
            slide(-translationSpeed);
            break;
            
        case 'p':
            _manager->pushActive();
            
            MeshViewInterface::postRedisplay();
            break;
            
        case 'u':
            _manager->popActive();
            
            MeshViewInterface::postRedisplay();
            break;
            
        case 'x':
            _manager->write();
            
            break;
    }
    
    glutPostRedisplay();
}

void MeshView::mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state== GLUT_DOWN)
    {
        pick(x, y);
        
        glutPostRedisplay();
    }
}

void MeshView::reshape(int w, int h)
{
    //_viewport[0] = w;
    //_viewport[1] = h;
    
    glutPostRedisplay();
}

void MeshView::postRedisplay()
{
    auto activeWindow = glutGetWindow();
    
    glutSetWindow(_windowId);
    glutPostRedisplay();
    
    glutSetWindow(activeWindow);
}
