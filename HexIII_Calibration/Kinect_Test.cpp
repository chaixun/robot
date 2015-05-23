#include "Kinect_Test.h"

bool Kinect::IsStepCaptureEnd;

Kinect::Kinect():interface(new pcl::OpenNIGrabber())
{
    frames_num = 0;
    save_one = false;
    Kinect::IsStepCaptureEnd = false;
}
Kinect::~Kinect()
{
    interface->stop();
    cout<<"Depth Close"<<endl;
}

void Kinect::start()
{
    interface->start();
    cout<<"Depth Open"<<endl;
}

void Kinect::capture(int *num)
{
    (*num)++;
    frames_num++;
    save_one = true;
    cout<<"NUM "<<*num<<endl;
}

void Kinect::pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    //Remove NAN
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    cloud1 = *cloud;
    cloud1.width = 640*480;
    cloud1.height = 1;

    if(save_one)
    {
        save_one = false;
        std::stringstream out;
        out<<frames_num-1;
        std::string dataname = "cloud" + out.str() + ".pcd";

        pcl::io::savePCDFileASCII(dataname,cloud1);

        cout<<"Capture "<<frames_num-1<<" end"<<endl;
        Kinect::IsStepCaptureEnd = true;
    }
}

void Kinect::viewcloud()
{
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&Kinect::pointcloud, this, _1);

    interface->registerCallback (f);
}

void Kinect::stop()
{
    interface->stop();
}
