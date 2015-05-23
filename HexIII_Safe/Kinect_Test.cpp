#include "Kinect_Test.h"

int Kinect::ControlCommand;
bool Kinect::IsCaptureEnd;

Kinect::Kinect():interface(new pcl::OpenNIGrabber())
{
    IsCapture = false;
    IsCaptureEnd = false;
    Kinect::ControlCommand = NoValidCommand;
    frames_num = 0;
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
    IsCapture = true;
    cout<<"Capture Step "<<*num<<" Beging !!!"<<endl;
}

void Kinect::pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if (IsCapture == true)
    {
        bool Obstacle = false;
        IsCapture = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr SensorPoint(new pcl::PointCloud<pcl::PointXYZ>);

        Eigen::Matrix4f transformation1;
        transformation1 << -1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        pcl::transformPointCloud(*cloud,*SensorPoint,transformation1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr RobotPoint(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f transformation2;
        transformation2 << 0.9995, 0.0134, -0.0273, 0.0224,
                -0.0304, 0.5120, -0.8584, 0.2026+0.85+0.038,
                0.0025, 0.8589, 0.5122, 0.5733,
                0, 0, 0, 1;
        //                transformation2 << 1, 0, 0, 0,
        //                        0, 1, 0, 0.73,
        //                        0, 0, 1, 0,
        //                        0, 0, 0, 1;
        pcl::transformPointCloud(*SensorPoint,*RobotPoint,transformation2);


        //Mapping GridMap
        Eigen::MatrixXf GridMap = Eigen::MatrixXf::Zero(120,120);
        Eigen::MatrixXf GridNum = Eigen::MatrixXf::Zero(120,120);
        for (size_t i = 0;i < RobotPoint->points.size(); ++i)
        {
            if(RobotPoint->points[i].x>-1.5&&RobotPoint->points[i].x<1.5&&
                    RobotPoint->points[i].z>0&&RobotPoint->points[i].z<3)
            {
                if(RobotPoint->points[i].y<0)
                {
                    RobotPoint->points[i].y = 0;
                }

                int m, n;
                n = floor(RobotPoint->points[i].x/0.025) + 60;
                m = floor(RobotPoint->points[i].z/0.025);

                //Mean
                GridMap(m,n) = (GridMap(m,n)*GridNum(m,n) + RobotPoint->points[i].y)/(GridNum(m,n) + 1);
                GridNum(m,n) = GridNum(m,n) + 1;

                //Max
                /*
        if (GridMap(m,n) < filtertransformed->points[i].y)
        {
            GridMap(m,n) = filtertransformed->points[i].y;
        }
        */
            }
        }
        //Judge Command

        float LeftHeight = 0, RightHeight = 0, MiddleHeight = 0;

        for(int i=40; i<=55; i++)
        {
            for(int j = 41;j<=54;j++)
            {
                if(GridMap(i,j)>=0.2)
                {
                    Obstacle = true;
                    RightHeight = RightHeight + GridMap(i,j);
                }
            }
        }

        for(int i=40; i<=55; i++)
        {
            for(int j = 54;j<=67;j++)
            {
                if(GridMap(i,j)>=0.2)
                {
                    Obstacle = true;
                    MiddleHeight = MiddleHeight + GridMap(i,j);
                }
            }
        }

        for(int i=40; i<=55; i++)
        {
            for(int j = 67;j<=80;j++)
            {
                if(GridMap(i,j)>=0.2)
                {
                    Obstacle = true;
                    LeftHeight = LeftHeight + GridMap(i,j);
                }
            }
        }

        if(Obstacle == false)
        {
            ControlCommand = NoValidCommand;
            cout<<"Robot Safe!!!"<<endl;
        }
        else
        {
            if(LeftHeight>RightHeight&&LeftHeight>MiddleHeight)
            {
                ControlCommand = TurnLeft;
                cout<<"Send Turn Right!!!"<<endl;
            }
            else if(RightHeight>LeftHeight&&RightHeight>MiddleHeight)
            {
                ControlCommand = TurnRight;
                cout<<"Send Turn Left!!!"<<endl;
            }
            else if(MiddleHeight>LeftHeight&&MiddleHeight>RightHeight)
            {
                ControlCommand = MoveForward;
                cout<<"Send Move Backward!!!"<<endl;
            }

        }

        std::stringstream out;
        out<<frames_num;
        std::string filename = "GridMap" + out.str() + ".txt";
        std::ofstream Gridmapfile(filename);
        if (Gridmapfile.is_open())
        {
            Gridmapfile<<GridMap<<endl;
        }

        IsCaptureEnd = true;
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
