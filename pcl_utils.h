/*
 *   pcl_utils.cpp - Some convenience functions for using data from
 *       Joint 2D-3D-Semantic Data for Indoor Scene Understanding
 *       in conjunction with Point Cloud Library (PCL)
 *
 *      Website: 3dsemantics.stanford.edu
 *      Paper: https://arxiv.org/pdf/1702.01105.pdf 
 *  
 *  Usage: Copy or include the code
 */
#ifndef SEMANTIC2D3D_UTILS_H
#define SEMANTIC2D3D_UTILS_H

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <pcl/common/eigen.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <vector>

namespace Utils
{

/** Gets the RGB_color from a pose file ptree and returns it**/
inline bool
getRGBcolor(const boost::property_tree::ptree &pt, std::vector<Eigen::Vector3d> &RGB)
{
    // Read in RGB_color
    Eigen::Vector3d rgb_point;
    int i = 0;

    boost::property_tree::ptree object = pt.get_child("object");
    BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, object)
    {
        boost::property_tree::ptree pt_tree = vtt.second;
        boost::property_tree::ptree RGB_color = pt_tree.get_child("RGB_color");
        BOOST_FOREACH (boost::property_tree::ptree::value_type &v, RGB_color)
        {
            boost::property_tree::ptree tree = v.second;
            BOOST_FOREACH (boost::property_tree::ptree::value_type &vt, tree)
            {
                rgb_point[i] = vt.second.get_value<float>();
                i++;
            }
            i = 0;
            // std::cout << rgb_point.transpose() << std::endl;
            RGB.push_back(rgb_point);
        }
    }

    return true;
}

inline bool
getXYZpoints(const boost::property_tree::ptree &pt, std::vector<Eigen::Vector3d> &XYZ)
{
    // Read in XYZ_color
    Eigen::Vector3d xyz_point;
    int i = 0;

    boost::property_tree::ptree object = pt.get_child("object");
    BOOST_FOREACH (boost::property_tree::ptree::value_type &vtt, object)
    {
        boost::property_tree::ptree pt_tree = vtt.second;
        boost::property_tree::ptree point = pt_tree.get_child("points");
        BOOST_FOREACH (boost::property_tree::ptree::value_type &v, point)
        {
            boost::property_tree::ptree tree = v.second;
            BOOST_FOREACH (boost::property_tree::ptree::value_type &vt, tree)
            {
                xyz_point[i] = vt.second.get_value<float>();
                i++;
            }
            i = 0;
            // std::cout << rgb_point.transpose() << std::endl;
            XYZ.push_back(xyz_point);
        }
    }

    return true;
}

/** Gets the camera rotation from a pose file ptree and returns it**/
// inline bool
// getXYZpoints(const boost::property_tree::ptree &pt, std::vector<Eigen::Vector3d> &Points)
// {
//     // Read in xyz_point
//     Eigen::Vector3d xyz_point;
//     int i = 0;
//     boost::property_tree::ptree XYZ_point = pt.get_child("points");
//     BOOST_FOREACH (boost::property_tree::ptree::value_type &v, XYZ_point)
//     {
//         boost::property_tree::ptree tree = v.second;
//         BOOST_FOREACH (boost::property_tree::ptree::value_type &vt, tree)
//         {
//             xyz_point[i] = vt.second.get_value<float>();
//             i++;
//         }
//         i = 0;
//         // std::cout << xyz_point.transpose() << std::endl;
//         Points.push_back(xyz_point);
//     }

//     return true;
// }

/** Gets the camera k from a pose file ptree and returns it**/
inline Eigen::Vector4d
getCameraKmatrix(const boost::property_tree::ptree &pt)
{
    // Read in camera translation
    float tr[9];
    int i = 0;
    boost::property_tree::ptree camera_k = pt.get_child("camera_k_matrix");
    // 遍历数组
    BOOST_FOREACH (boost::property_tree::ptree::value_type &v, camera_k)
    {
        boost::property_tree::ptree tree = v.second;
        BOOST_FOREACH (boost::property_tree::ptree::value_type &vt, tree)
        {
            tr[i] = vt.second.get_value<float>();
            i++;
        }
    }
    Eigen::Vector4d k(tr[0], tr[4], tr[2], tr[5]);
    return k;
}

/** Gets the camera_rt_matrix from a pose file ptree and returns it**/
inline Eigen::Isometry3d
getCameraRTmatrix(const boost::property_tree::ptree &pt)
{
    // Read in camera translation
    float tr[12];
    int i = 0;
    boost::property_tree::ptree camera_rt_matrix = pt.get_child("camera_rt_matrix");
    // 遍历数组
    BOOST_FOREACH (boost::property_tree::ptree::value_type &v, camera_rt_matrix)
    {
        boost::property_tree::ptree tree = v.second;
        BOOST_FOREACH (boost::property_tree::ptree::value_type &vt, tree)
        {
            tr[i] = vt.second.get_value<float>();
            std::cout << tr[i] << std::endl;
            i++;
        }
    }
    Eigen::Isometry3d T;
    Eigen::Matrix3d R;
    T.matrix() << tr[0], tr[1], tr[2], tr[3],
        tr[4], tr[5], tr[6], tr[7],
        tr[8], tr[9], tr[10], tr[11],
        0, 0, 0, 1;
    // T.rotate(Eigen::Quaterniond(R));
    // T.pretranslate(Eigen::Vector3d(tr[3],tr[7],tr[11]));
    return T;
}

/** Load in the json pose files into a boost::ptree **/
inline boost::property_tree::ptree
loadPoseFile(const std::string &json_filename)
{
    // Read in view_dict
    boost::property_tree::ptree pt;
    std::ifstream in(json_filename);
    std::stringstream buffer;
    buffer << in.rdbuf();
    read_json(buffer, pt);
    return pt;
}

/** Debugging function to print out a boost::ptree **/
void print(boost::property_tree::ptree const &pt)
{
    using boost::property_tree::ptree;
    ptree::const_iterator end = pt.end();
    for (ptree::const_iterator it = pt.begin(); it != end; ++it)
    {
        std::cout << it->first << ": " << it->second.get_value<std::string>() << std::endl;
        print(it->second);
    }
}
} // namespace Utils

#endif // #ifndef SEMANTIC2D3D_UTILS_H
