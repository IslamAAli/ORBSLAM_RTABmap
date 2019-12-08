#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/organized_pointcloud_conversion.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/calib3d.hpp>

using namespace std;
using namespace cv;


pcl::PointCloud<pcl::PointXYZ> FillPointCloud(Mat XYZ_points)
{
        pcl::PointCloud<pcl::PointXYZ> pointCloud;

        pointCloud.width = XYZ_points.cols; //Dimensions must be initialized to use 2-D indexing
        pointCloud.height = XYZ_points.rows;
        //pointCloud.resize(pointCloud.width*pointCloud.height);

        for (int i = 0; i < XYZ_points.rows; i ++)
        {
                for (int j= 0; j < XYZ_points.cols; j ++)
                {
                        Vec3f point = XYZ_points.at<Vec3f>(i, j);

                        pcl::PointXYZ p;
                        p.x = point[0];
                        p.y = point[1];
                        p.z = point[2];
                        pointCloud.points.push_back(p);

                }
        }
        return pointCloud;
}

int main (int argc, char** argv)
{
    Mat im_left     = imread("../data/kitti_left.png", cv::IMREAD_UNCHANGED);
    Mat im_right    = imread("../data/kitti_right.png", cv::IMREAD_UNCHANGED);

    im_left.convertTo(im_left, CV_8UC1);
    im_right.convertTo(im_right, CV_8UC1);

    Ptr<StereoBM> sbm = StereoBM::create(16, 5);
//    sbm->setPreFilterCap(63);
//    sbm->setPreFilterSize(5);
//    sbm->setPreFilterType(0);
//    sbm->setSmallerBlockSize(0);
//    sbm->setTextureThreshold(100);
//    sbm->setUniquenessRatio(5);
//    sbm->setBlockSize(5);
//    sbm->setDisp12MaxDiff(1);
//    sbm->setMinDisparity(-39);
//    sbm->setNumDisparities(112);
//    sbm->setSpeckleRange(20);
//    sbm->setSpeckleWindowSize(0);

    Mat disp,disp8;

    sbm->compute(im_left,im_right,disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);


    std::cout<< "Computed Disparity ..." <<endl;

    int dispWidth = disp8.size().width;
    int dispHeight= disp8.size().height;
    double kittiFocalLength = 645.24 ;  // to be revised (in pixels)
    double kittiBaseLine = 0.54;

    std::cout<< "Disparity Normalization ... " <<endl;

    Mat XYZ_points;
    Mat Q = Mat::zeros(Size(4, 4), CV_64FC1);
    Q.at<double>(0,0)=1.0;
    Q.at<double>(0,1)=0.0;
    Q.at<double>(0,2)=0.0;
    Q.at<double>(0,3)=-1*(dispWidth/2); //cx
    Q.at<double>(1,0)=0.0;
    Q.at<double>(1,1)=1.0;
    Q.at<double>(1,2)=0.0;
    Q.at<double>(1,3)=-1*(dispHeight/2);  //cy
    Q.at<double>(2,0)=0.0;
    Q.at<double>(2,1)=0.0;
    Q.at<double>(2,2)=0.0;
    Q.at<double>(2,3)=kittiFocalLength;  //Focal
    Q.at<double>(3,0)=0.0;
    Q.at<double>(3,1)=0.0;
    Q.at<double>(3,2)=1.0/kittiBaseLine;    //1.0/BaseLine
    Q.at<double>(3,3)=0.0;    //cx - cx'

    std::cout<< "Q Matrix Generated ... " <<endl;

    reprojectImageTo3D(disp8, XYZ_points, Q, true, -1);


    std::cout<< "3D Points Generated ... " <<endl;


    // fill the points into a point cloud and visualize it
    pcl::PointCloud<pcl::PointXYZ> cloud = FillPointCloud(XYZ_points);

    std::cout<< "Point Cloud filled ... " <<endl;

//    // code to copy contents of matrix to vec
//    std::vector<uint16_t> vec_image;
//    std::vector<uchar> dummy;

//    std::cout<< " test2" <<endl;
//    std::cout<< disp8.rows << " " << disp8.cols << endl;

//    for (int i=0; i< disp8.rows-1 ; i++)
//    {
//        for (int j=0; j< disp8.cols-1 ; j++)
//        {
//            vec_image.push_back(disp8.at<uint16_t>(i,j));
//            cout << disp8.rows << "  " << disp8.cols << "  " << i << "  " << j << endl;
//        }
//    }

//    std::cout<< " test3" <<endl;

//    // convert depth image to pointcloud
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::OrganizedConversion<pcl::PointXYZ,false>::convert(vec_image,dummy, false,
//                                                               dispWidth-1, dispHeight-1,
//                                                               kittiFocalLength, 0, 1,
//                                                               *cloud);



    Mat res;
    vconcat(im_left, im_right, res);
    //vconcat(res, disp8, res);

    //imshow("input", res);
    imshow("disparity", disp8);

    // visualize cloud
    pcl::io::savePCDFileASCII("kitti_cloud.pcd", cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("kitti_cloud.pcd", *cloudPtr);

    std::cout<< "Point Cloud Loaded ... " <<endl;

    pcl::visualization::CloudViewer viewer1("KITTI Point Cloud");
    viewer1.showCloud(cloudPtr);
    while (!viewer1.wasStopped())
    {
    }

    waitKey(0);

    return 0;
}
