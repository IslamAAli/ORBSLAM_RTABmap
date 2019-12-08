// hw2_t1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
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

			/*p.z = p.z / 1000;
			p.x = p.x / 1000;
			p.y = p.y / 1000;*/

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
	pcl::PointCloud<pcl::PointXYZ> img_2_PC;
	pcl::PointCloud<pcl::PointXYZ> img_3_PC;

	cv::Mat I1 = cv::imread("466046.png", cv::IMREAD_UNCHANGED);
	cv::Mat I2 = cv::imread("498096.png", cv::IMREAD_UNCHANGED);
	cv::Mat I3 = cv::imread("528267.png", cv::IMREAD_UNCHANGED);

	img_1_PC = RGBDtoPCL(I1, fx, fy, cx, cy);
	img_2_PC = RGBDtoPCL(I2, fx, fy, cx, cy);
	img_3_PC = RGBDtoPCL(I3, fx, fy, cx, cy);


	//pcl::io::savePCDFileASCII("c1_min_2.pcd", img_1_PC);
	//pcl::io::savePCDFileASCII("c2_min_2.pcd", img_2_PC);
	//pcl::io::savePCDFileASCII("c3_min_2.pcd", img_3_PC);


	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);

	cout << "Loading C1" << endl;
	pcl::io::loadPCDFile("c1_min_2.pcd", *cloud1);
	cout << "Loading C2" << endl;
	pcl::io::loadPCDFile("c2_min_2.pcd", *cloud2);
	cout << "Loading C3" << endl;
	pcl::io::loadPCDFile("c3_min_2.pcd", *cloud3);

	

	cout << "ICP 1&2 ... " << endl;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	cout << "ICP 1&2 source" << endl;
	icp.setInputSource(cloud1);
	cout << "ICP 1&2 dst" << endl;
	icp.setInputTarget(cloud2);
	
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(0.05);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations(500);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon(1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon(1);


	cout << "ICP 1&2 Run" << endl;
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	//=============================================================================

	cout << "ICP 2&3 ... " << endl;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_2;
	cout << "ICP 2&3 source" << endl;
	icp_2.setInputSource(cloud2);
	cout << "ICP 2&3 dst" << endl;
	icp_2.setInputTarget(cloud3);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp_2.setMaxCorrespondenceDistance(0.05);
	// Set the maximum number of iterations (criterion 1)
	icp_2.setMaximumIterations(500);
	// Set the transformation epsilon (criterion 2)
	icp_2.setTransformationEpsilon(1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp_2.setEuclideanFitnessEpsilon(1);


	cout << "ICP 2&3 Run" << endl;
	pcl::PointCloud<pcl::PointXYZ> Final_2;
	icp_2.align(Final_2);
	std::cout << "has converged:" << icp_2.hasConverged() << " score: " <<
		icp_2.getFitnessScore() << std::endl;
	std::cout << icp_2.getFinalTransformation() << std::endl;*/

	pcl::io::savePCDFileASCII("c1_main.pcd", img_1_PC);
	pcl::io::savePCDFileASCII("c2_main.pcd", img_2_PC);
	pcl::io::savePCDFileASCII("c3_main.pcd", img_3_PC);
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("c1_main.pcd", *cloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("c2_main.pcd", *cloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("c3_main.pcd", *cloud3);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
	viewer1.showCloud(cloud1);
	while (!viewer1.wasStopped())
	{
	}

	pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer");
	viewer2.showCloud(cloud2);
	while (!viewer2.wasStopped())
	{
	}

	pcl::visualization::CloudViewer viewer3("Simple Cloud Viewer");
	viewer3.showCloud(cloud3);
	while (!viewer3.wasStopped())
	{
	}
	
	return (0);
}