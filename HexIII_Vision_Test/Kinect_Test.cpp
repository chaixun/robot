#include "Kinect_Test.h"

double Kinect::CurrentHeight[4];
bool Kinect::IsCaptureEnd;
int Kinect::ControlCommand;
int Kinect::leftedge_z;
int Kinect::rightedge_z;
int Kinect::near_leftedge_z;
int Kinect::near_rightedge_z;
int Kinect::far_leftedge_z;
int Kinect::far_rightedge_z;
int Kinect::leftedge_x;
int Kinect::rightedge_x;
int Kinect::Terrain;

Kinect::Kinect():interface(new pcl::OpenNIGrabber())
{
    IsCapture = false;
    IsCaptureEnd = false;
    frames_num = 0;
    leftedge_z = 0;
    rightedge_z = 0;
    near_leftedge_z = 0;
    near_rightedge_z = 0;
    far_leftedge_z = 0;
    far_rightedge_z = 0;
    leftedge_x = 0;
    rightedge_x = 0;
    Terrain = FlatTerrain;
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
        bool positive = false;
        bool negative = false;
        //bool Obstacle = false;
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
                -0.0304, 0.5120, -0.8584, 0.2026 + 0.038,
                0.0025, 0.8589, 0.5122, 0.5733,
                0, 0, 0, 1;
        // 0.2026 + 0.85 + 0.038 + 0.2
        //        transformation2 << 0.9996, 0.0233, -0.0137, 0,
        //                -0.0270, 0.8622, -0.5059, 0.0296,
        //                0, 0.5061, 0.8625, 0,
        //                0, 0, 0, 1;
        pcl::transformPointCloud(*SensorPoint,*RobotPoint,transformation2);


        //Mapping GridMap

        Eigen::MatrixXf GridMap = Eigen::MatrixXf::Zero(120,120);
        Eigen::MatrixXf GridNum = Eigen::MatrixXf::Zero(120,120);
        for (size_t i = 0;i < RobotPoint->points.size(); ++i)
        {
            if(RobotPoint->points[i].x>-1.5&&RobotPoint->points[i].x<1.5&&
                    RobotPoint->points[i].z>0&&RobotPoint->points[i].z<3)
            {
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

        //Judge Terrain

        for(int k = 26; k <= 50; k++)
        {
            if(GridMap(k + 1, 60) - GridMap(k, 60) > 0.05)
            {
                positive = true;
            }

            if(GridMap(k + 1, 60) - GridMap(k, 60) < -0.05)
            {
                negative = true;
            }
        }

        if(positive == true&& negative == false)
        {
            Terrain = StepUpTerrain;
        }
        if(positive == false&& negative == true)
        {
            Terrain = StepDownTerrain;
        }
        if(positive == true&& negative == true)
        {
            Terrain = DitchTerrain;
        }
        if(positive == false&& negative == false)
        {
            Terrain = FlatTerrain;
        }

        //Find Edge

        /*Find Edge Along Z*/
        for(int m = 26; m <= 50; m++)
        {
            if(abs(GridMap(m+1, 48)-GridMap(m, 48)) > 0.07)
            {
                rightedge_z = m + 1;
            }

            if(abs(GridMap(m+1, 72)-GridMap(m, 72)) > 0.07)
            {
                leftedge_z = m + 1;
            }

            if(Terrain == DitchTerrain)
            {
                if(GridMap(m+1, 48)-GridMap(m, 48) < -0.07)
                {
                    near_rightedge_z = m;
                }
                if(GridMap(m+1, 48)-GridMap(m, 48) > 0.07)
                {
                    far_rightedge_z = m + 1;
                }
                if(GridMap(m+1, 72)-GridMap(m, 72) < -0.07)
                {
                    near_leftedge_z = m;
                }
                if(GridMap(m+1, 72)-GridMap(m, 72) > 0.07)
                {
                    far_leftedge_z = m + 1;
                }
            }
        }

        /*Find Edge Along X*/
        for(int k = 0; k < 58; k++)
        {
            if(abs(GridMap(39, 60-k-1) - GridMap(39, 60-k)) > 0.08 )
            {
                rightedge_x = 60 - k;
            }
            if(abs(GridMap(39, 60+k+1) - GridMap(39, 60+k)) > 0.08 )
            {
                leftedge_x = 60 + k;
            }
        }

        //Judge Command

        CurrentHeight[0] = (GridMap(39,43) + GridMap(39,44) + GridMap(40,43) + GridMap(40,44))/4;
        CurrentHeight[1] = (GridMap(39,48) + GridMap(39,49) + GridMap(40,48) + GridMap(40,49))/4;
        CurrentHeight[2] = (GridMap(39,78) + GridMap(39,79) + GridMap(40,78) + GridMap(40,79))/4;
        CurrentHeight[3] = (GridMap(39,72) + GridMap(39,73) + GridMap(40,72) + GridMap(40,73))/4;

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
