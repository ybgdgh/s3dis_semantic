#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <string>
#include <Eigen/Core>
#include <unistd.h>
#include <fstream>
#include <cstring>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>

#include "pcl_utils.h"


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;


int main(int argc, char **argv)
{

    string filename = "point_data/";

    vector<String> files_json; //存放文件路径

    glob(filename, files_json, true);
    // glob(filename_rgb, files_rgb, true);
    // glob(filename_depth, files_depth, true);

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    // PointCloud<PointXYZRGB>::Ptr cloud_sum(new PointCloud<PointXYZRGB>);


    // for (int i = 0; i < files_json.size(); i++)
    // {
    //     cout << "name : " << files_json[i] << endl;
    //     pcl::io::loadPCDFile(files_json[i], *cloud);
    //     *cloud_sum = *cloud_sum + *cloud;
    //     cloud->points.clear();
    //     cout << "sum " << i << " / " << files_json.size() <<  "done" << endl;
    // }
    // pcl::io::savePCDFile("cloud_sum.pcd", *cloud_sum);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0, 0, 0);

    for (int i = 2; i < files_json.size(); i++)
    {

        //     cout << "read name :" << i << " : " << files[i/3] << endl;

        boost::property_tree::ptree pose_pt = Utils::loadPoseFile(files_json[i]);

        string name = pose_pt.get<string>("name");

        cout << "name : " << name << endl;
        // string global_name = pose_pt.get<string>("global_name");

        std::vector<Eigen::Vector3d> RGB_color;
        if (!Utils::getRGBcolor(pose_pt, RGB_color))
            ;

        std::vector<Eigen::Vector3d> points;
        if (!Utils::getXYZpoints(pose_pt, points))
            ;
        cout << "read date done.." << endl;
        // cout << "global_name : " << global_name << endl;
        // cout << "RGB_color_size : " << RGB_color.size() << endl;
        // cout << "points_size : " << points.size() << endl;

        for (int j = 0; j < RGB_color.size(); j++)
        {
            pcl::PointXYZRGB point;

            point.x = points[j](0);
            point.y = points[j](1);
            point.z = points[j](2);

            point.b = RGB_color[j](2);
            point.g = RGB_color[j](1);
            point.r = RGB_color[j](0);

            cloud->push_back(point);
        }
        cout << i << "point number :" << cloud->points.size() << endl;

        String ss = "pointcloud_" + name + ".pcd";
        pcl::io::savePCDFile(ss, *cloud);

        // 清除数据并退出
        cloud->points.clear();
        cout << "Point cloud saved." << endl;
        cout << "next pcd reading..." << endl;
    }

    cloud->width = 1;
    cloud->height = cloud->points.size();

    depth_cloud->width = 1;
    depth_cloud->height = cloud->points.size();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_cloud(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem(1.0);

    // cv::imwrite("vignette.jpeg", test);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    cv::waitKey(0);
    viewer->removeAllPointClouds();


    return 0;
}
