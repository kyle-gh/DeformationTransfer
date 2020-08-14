project(deform)

cmake_minimum_required(VERSION 3.17)

set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# setup GLUT
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

# setup OpenGL
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})

# setup OpenMesh
set(OPENMESH_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../OpenMesh/lib)
find_package(OpenMesh REQUIRED)
include_directories(${OPENMESH_INCLUDE_DIRS})

set(ENV{EIGEN3_ROOT} ${CMAKE_CURRENT_SOURCE_DIR}/../eigen)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE true)
find_package(CGAL)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set_property(
    DIRECTORY
    APPEND PROPERTY COMPILE_DEFINITIONS _USE_MATH_DEFINES
)

include_directories(src)
include_directories(src/corrtool)
include_directories(src/transfer)
include_directories(src/shared)
include_directories(src/shared/correspondence)
include_directories(src/shared/transfer)

file(GLOB_RECURSE shared_sources src/shared/*.c*)
file(GLOB_RECURSE shared_headers src/shared/*.h*)

add_executable(transfer
        src/transfer/Transfer.cpp
        ${shared_sources}
        ${shared_headers}
        )

target_link_libraries(transfer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES} ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT} CGAL::CGAL)

add_executable(cortool
        src/corrtool/Camera.cpp
        src/corrtool/Camera.h
        src/corrtool/CorrespondenceManager.cpp
        src/corrtool/CorrespondenceManager.h
        src/corrtool/corrtool.cpp
        src/corrtool/gl.h
        src/corrtool/MeshView.cpp
        src/corrtool/MeshView.h
        ${shared_sources}
        ${shared_headers}
        )

target_link_libraries(cortool ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OPENMESH_LIBRARIES} ${CMAKE_DL_LIBS}  ${CMAKE_THREAD_LIBS_INIT} CGAL::CGAL)