#ifndef KINECT_TEST_H
#define KINECT_TEST_H

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>


using namespace std;
using namespace cv;
using namespace pcl;


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
    static bool IsStepCaptureEnd;
private:
    pcl::OpenNIGrabber* interface;
    int frames_num;
    bool save_one;
};

#endif // KINECT_TEST_H
