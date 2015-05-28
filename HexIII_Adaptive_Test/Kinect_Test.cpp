#include "Kinect_Test.h"

double Kinect::CurrentHeight[4];
bool Kinect::IsCaptureEnd;

Kinect::Kinect():interface(new pcl::OpenNIGrabber())
{
    IsCapture = false;
    IsCaptureEnd = false;
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
                -0.0304, 0.5120, -0.8584, 0.2026+0.85+0.038 + 0.2,
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

                Position temp_p;
                temp_p.x = RobotPoint->points[i].x;
                temp_p.y = RobotPoint->points[i].y;
                temp_p.z = RobotPoint->points[i].z;

                //Mean
                GridMap(m,n) = (GridMap(m,n)*GridNum(m,n) + RobotPoint->points[i].y)/(GridNum(m,n) + 1);
                GridNum(m,n) = GridNum(m,n) + 1;

                Map[m][n].height = GridMap(m,n);
                Map[m][n].pointcloud.push_back(temp_p);

                //Max
                /*
        if (GridMap(m,n) < filtertransformed->points[i].y)
        {
            GridMap(m,n) = filtertransformed->points[i].y;
        }
        */
            }
        }

        for(int i = 0; i < 120; i++)
        {
            for(int j = 0; j < 120; j++)
            {
                int num = Map[i][j].pointcloud.size();
                double A[num][3];
                double b[num][1];

                for(int i = 0; i < num; i++)
                {
                    A[i][0] = Map[i][j].pointcloud.at(i).x;
                    A[i][1] = Map[i][j].pointcloud.at(i).y;
                    A[i][2] = Map[i][j].pointcloud.at(i).z;
                    b[i][0] = 1;
                }
                lapack_int info;

                info =  LAPACKE_dgels(LAPACK_ROW_MAJOR,'N',num,3,1,*A,3,*b,1);
                if(*b[0] < 0)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        *b[i] = -*b[i];
                    }
                }
                Map[i][j].Plane_Para[0] = *b[0];
                Map[i][j].Plane_Para[1] = *b[1];
                Map[i][j].Plane_Para[2] = *b[2];
                Map[i][j].Plane_Para[3] = -*b[3];

                for(int i = 0; i < num; i++)
                {
                    Map[i][j].Flatness = abs(Map[i][j].Plane_Para[0]*Map[i][j].pointcloud.at(i).x +
                            Map[i][j].Plane_Para[1]*Map[i][j].pointcloud.at(i).y +
                            Map[i][j].Plane_Para[2]*Map[i][j].pointcloud.at(i).z + Map[i][j].Plane_Para[3])
                            /sqrt(Map[i][j].Plane_Para[0]*Map[i][j].Plane_Para[0] + Map[i][j].Plane_Para[1]*Map[i][j].Plane_Para[1]
                            + Map[i][j].Plane_Para[2]*Map[i][j].Plane_Para[2]) + Map[i][j].Flatness;
                }
            }
        }

        //Judge Command

        CurrentHeight[0] = (GridMap(39,43) + GridMap(39,44) + GridMap(40,43) + GridMap(40,44))/4;
        CurrentHeight[1] = (GridMap(39,48) + GridMap(39,49) + GridMap(40,48) + GridMap(40,49))/4;
        CurrentHeight[2] = (GridMap(39,78) + GridMap(39,79) + GridMap(40,78) + GridMap(40,79))/4;
        CurrentHeight[3] = (GridMap(39,72) + GridMap(39,73) + GridMap(40,72) + GridMap(40,73))/4;

        for(int i = 0;i < 4; i++)
        {
            if(CurrentHeight[i] > 0.23)
            {
                CurrentHeight[i] = 0.23;
            }

            if(CurrentHeight[i] < 0)
            {
                CurrentHeight[i] = 0;
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
