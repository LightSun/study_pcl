#ifndef STUDYIO_H
#define STUDYIO_H

#include <string>
#include "compile_config.h"

class StudyIO
{
public:
    static void stl2pcd(const std::string& stdPath, const std::string& savePcdPath);
    static void obj2pcd(const std::string& objPath, const std::string& savePcdPath);
    static void ply2pcd(const std::string& plyPath, const std::string& savePcdPath);

#ifdef COMPILE_WITH_BLAS
    static void pcd2las(const std::string& objPath, const std::string& savePcdPath);
    static void las2pcd(const std::string& objPath, const std::string& savePcdPath);
#endif
};

#endif // STUDYIO_H
