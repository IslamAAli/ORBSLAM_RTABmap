#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <opencv2/opencv.hpp>

#define _CRT_SECURE_NO_WARNINGS

pcl::PointCloud<pcl::PointXYZ> RGBDtoPCL(cv::Mat depth_image, float fx, float fy, float cx, float cy)
{
        pcl::PointCloud<pcl::PointXYZ> pointCloud;

        float factor = 5000; //1;

        depth_image.convertTo(depth_image, CV_32FC1); // convert the image data to float type

        //cv::imshow("te",depth_image);
        //cv::waitKey(0);

        if (!depth_image.data) {
                std::cerr << "No depth data!!!" << std::endl;
                exit(EXIT_FAILURE);
        }

        int c = 0;
        pointCloud.width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing
        pointCloud.height = depth_image.rows;
        //pointCloud.resize(pointCloud.width*pointCloud.height);

        for (int v = 0; v < depth_image.rows; v ++)
        {
                for (int u = 0; u < depth_image.cols; u ++)
                {
                        c++;
                        //cout << c << endl;
                        float Z = depth_image.at<float>(v, u) / factor;

                        pcl::PointXYZ p;
                        p.z = Z;
                        p.x = (u - cx) * Z / fx;
                        p.y = (v - cy) * Z / fy;

                        //cout << p.x << "\t\t" << p.y << "\t\t" << p.z << endl;

                        pointCloud.points.push_back(p);

                }
        }
        return pointCloud;
}


int
main(int argc, char** argv)
{
        const float fx = 517.3;
        const float fy = 516.5;
        const float cx = 318.6;
        const float cy = 255.3;

        pcl::PointCloud<pcl::PointXYZ> img_1_PC;

        cv::Mat I1 = cv::imread("../data/466046.png", cv::IMREAD_UNCHANGED);

        img_1_PC = RGBDtoPCL(I1, fx, fy, cx, cy);

        pcl::io::savePCDFileASCII("c1_main.pcd", img_1_PC);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile("c1_main.pcd", *cloud1);

        pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
        viewer1.showCloud(cloud1);
        while (!viewer1.wasStopped())
        {}

        return (0);
}
