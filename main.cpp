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


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;


int main(int argc, char **argv)
{
    // read picture
    string res;
    stringstream ss;

    string filename = "pic";

    // vector<String> files; //存放文件路径

    // glob(filename, files, true);

    // for (int i = 0; i < files.size(); i++)
    //     cout << "read name :" << i << " : " << files[i/3] << endl;

    // 打开文件用于写，若文件不存在就创建它

    // fstream f_shiji("shijiweizi.txt", ios::in);

    // fstream f_lilun("lilunweizi.txt", ios::in);
    

    // if (!f_shiji || !f_lilun)
    //     return 0;



/*
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ("rgb_textures/5acd38aa86ea42028a265d20947754f5.obj", mesh);

    pcl::TextureMesh mesh2;
    pcl::io::loadOBJFile("rgb_textures/5acd38aa86ea42028a265d20947754f5.obj",mesh2);
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0, 0, 0);

    
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    // viewer->addTextureMesh(mesh2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    cv::waitKey(0);
    viewer->removeAllPointClouds();
*/ 
/*
    // 内参 459.8806,0,323.1796;0,460.0172,265.2708;0,0,1
    double fx = 459.8806, fy = 460.0172, cx = 323.1796, cy = 265.2708;

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    PointCloud<PointXYZRGB>::Ptr depth_cloud(new PointCloud<PointXYZRGB>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0, 0, 0);
    
    double b=35.64;
    double x=0,y=0,z=0;
    double qw=0,qx=0,qy=0,qz=0;
    for (int i = 0; i < 9; i=i+3)
    {
      cv::Mat origen_rgb = cv::imread(files[i], -1);
      cout << "read name origen_rgb:" << files[i] << endl;
      cv::Mat origen_depth = cv::imread(files[i+1], -1);
      cout << "read name origen_depth:" << files[i+1] << endl;
      
      f_lilun >> x >> y >> z >> qx >> qy >> qz >> qw ;
      
      Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q;
      q.x()=qx;
      q.y()=qy;
      q.z()=qz;
      q.w()=qw;
      
      T.rotate ( q );
      T.pretranslate ( Eigen::Vector3d ( x,y,z ) );
      cout << "T :" << T.matrix() << endl;

      for (int v = 0; v < origen_depth.rows; v++)
	  for (int u = 0; u < origen_depth.cols; u++)
	  {

	      pcl::PointXYZRGB point;
	      // 根据双目模型计算 point 的位置
	      double x = (u - cx) / fx;
	      double y = (v - cy) / fy;
// 	      double depth = (double)(fx * b) /(origen_depth.at<uchar>(v, u)*2+1);
	      double depth = (255-origen_depth.at<uchar>(v, u))*20+300;
	      
	      point.x = x * depth;
	      point.y = y * depth;
	      point.z = depth;
// 	      cout << "point depth :" << depth << endl;
	      
	      Eigen::Vector3d point_last(point.x,point.y,point.z);
	      Eigen::Vector3d point_next = T*point_last;
	      
	      point.x = point_next(0);
	      point.y = point_next(1);
	      point.z = point_next(2);

	      // 从rgb图像中获取它的颜色
	      // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
	      point.b = origen_rgb.ptr<uchar>(v)[u * 3];
	      point.g = origen_rgb.ptr<uchar>(v)[u * 3 + 1];
	      point.r = origen_rgb.ptr<uchar>(v)[u * 3 + 2];

	      cloud->push_back(point);
	      
	  }
	  cout << "point number :" << cloud->points.size() << endl;
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

    // cv::waitKey(0);
    pcl::io::savePCDFile("pointcloud_godak.pcd", *cloud);

    cv::waitKey(0);
    // 清除数据并退出
    cloud->points.clear();
    cout << "Point cloud saved." << endl;

    // f_shiji.close();
    // f_lilun.close();
*/
    return 0;
}


