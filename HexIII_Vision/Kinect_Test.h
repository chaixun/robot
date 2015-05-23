#ifndef KINECT_TEST_H
#define KINECT_TEST_H

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;

enum Command
{
    NoValidCommand = 0,
    MoveForward = 1,
    MoveBackWard = 2,
    TurnLeft = 3,
    TurnRight = 4,
};


class Kinect
{
public:
    Kinect();
    ~Kinect();
    void capture(int *num);
    void pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
    void viewcloud();
    void start();
    void stop();
    static double CurrentHeight[4];
    static int ControlCommand;
    static bool IsCaptureEnd;
private:
    pcl::OpenNIGrabber* interface;
    int frames_num;
    bool IsCapture;
};

#endif // KINECT_TEST_H
