//
//  gl.h
//  CorrTool
//
//  Created by Kyle on 12/20/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef gl_h
#define gl_h

#ifndef __APPLE__
#include <stdlib.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#define GL_SILENCE_DEPRECATION
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#endif

#endif /* gl_h */
