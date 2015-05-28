#ifndef IMAGE_H
#define IMAGE_H

#include <vector>

using namespace std;

class Position
{
public:
    float x, y, z;
};

class Image
{
public:
    Image():height(0.0),Roughness(0.0){}
    float height;
    vector<Position> pointcloud;
    float Plane_Para[4];
    float Roughness;
};


#endif // IMAGE_H
