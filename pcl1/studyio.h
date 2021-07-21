#ifndef STUDYIO_H
#define STUDYIO_H

#include <string>

class StudyIO
{
public:
    static void stl2pcd(const std::string& stdPath, const std::string& savePcdPath);
    static void obj2pcd(const std::string& objPath, const std::string& savePcdPath);
};

#endif // STUDYIO_H
