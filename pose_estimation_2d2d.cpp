#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>
#include <sstream>
#include<stdlib.h>
// #include "extra.h" // use this if in OpenCV2
using namespace std;
using namespace cv;

/****************************************************
 * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
 * **************************************************/

//方法一: 利用STL自己实现split 函数(常用，简单，直观)
//原型: vector<string> split(const string &s, const string &seperator);
vector<string> split(const string &s, const string &seperator){
    vector<string> result;
    typedef string::size_type string_size;
    string_size i = 0;
    while(i != s.size()){
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while(i != s.size() && flag == 0){
            flag = 1;
            for(string_size x = 0; x < seperator.size(); ++x)
                if(s[i] == seperator[x])
                {
                    ++i;
                    flag = 0;
                    break;
                }
        }
        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while(j != s.size() && flag == 0){
            for(string_size x = 0; x < seperator.size(); ++x)
                if(s[j] == seperator[x]){
                    flag = 1;
                    break;
                }
            if(flag == 0)
                ++j;
        }
        if(i != j){
            result.push_back(s.substr(i, j-i));
            i = j;
        }
    }
    return result;
}



void find_feature_matches (
        const Mat& img_1, const Mat& img_2,
        std::vector<KeyPoint>& keypoints_1,
        std::vector<KeyPoint>& keypoints_2,
        std::vector< DMatch >& matches );

void pose_estimation_2d2d (
        std::vector<KeyPoint> keypoints_1,
        std::vector<KeyPoint> keypoints_2,
        std::vector< DMatch > matches,
        Mat& R, Mat& t );

void pose_estimation_2d2d_s (
        std::vector<KeyPoint> keypoints_1,
        std::vector<KeyPoint> keypoints_2,
        std::vector< DMatch > matches,
        Mat& R, Mat& t ,int& E_empty,int& RANSAC_outlier, int& cam_front,double& cam_front_raito);


void pose_estimation_2d2d_ss ( std::vector<KeyPoint> keypoints_1,
                               std::vector<KeyPoint> keypoints_2,
                               std::vector< DMatch > matches,
                               Mat& R1, Mat& R2, Mat& t ,int& E_empty,int& RANSAC_outlier, int& cam_front,double& cam_front_raito);
// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

int main ( int argc, char** argv )
{
    ifstream myfile("../duantouImgUndistortsuo.txt"); //DTL
    ofstream fout_c("../dtl_image_rt.txt",ios::app);//DTL

//    ifstream myfile("../left.txt"); //SLL
//    ofstream fout_c("../sll_image_rt.txt",ios::app);//SLL


    string img_1_name,img_temp;
    if (!myfile.is_open())
    {
        cout << "Open file txt failure" << endl;
    }

    int num=0;

    while(getline(myfile,img_1_name))//    while(1)
    {
        num++;

        if(num<2)
        {
            img_temp=img_1_name;
            continue;
        }


    //-- 读取图像
    Mat img_1 = imread ("../duantouImgUndistortsuo/"+img_temp, CV_LOAD_IMAGE_COLOR );//dtl
    Mat img_2 = imread ("../duantouImgUndistortsuo/"+img_1_name, CV_LOAD_IMAGE_COLOR );//dtl
    cout<<"../duantouImgUndistortsuo/"+img_temp<<" "<<"../duantouImgUndistortsuo/"+img_1_name<<endl;//dtl

//        Mat img_1 = imread ("../left/"+img_temp, CV_LOAD_IMAGE_COLOR );//sll
//        Mat img_2 = imread ("../left/"+img_1_name, CV_LOAD_IMAGE_COLOR );//sll
//        cout<<"../left/"+img_temp<<" "<<"../left/"+img_1_name<<endl;//sll

//    imshow("w1 ",img_1);
//    imshow("w2 ",img_2);
//    cv::waitKey(30);
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );

//    int E_empty;int RANSAC_outlier; int cam_front;double cam_front_raito;
//    pose_estimation_2d2d_s (keypoints_1, keypoints_2, matches, R, t,E_empty,RANSAC_outlier,cam_front,cam_front_raito);

    //TODO


    cout<<"num  "<<keypoints_1.size()<<" "<<keypoints_2.size()<<endl;

    //-- 验证E=t^R*scale
    Mat t_x = ( Mat_<double> ( 3,3 ) <<
                                     0,                      -t.at<double> ( 2,0 ),     t.at<double> ( 1,0 ),
            t.at<double> ( 2,0 ),      0,                      -t.at<double> ( 0,0 ),
            -t.at<double> ( 1.0 ),     t.at<double> ( 0,0 ),      0 );

    cout<<"t^R="<<endl<<t_x*R<<endl;

//    //-- 验证对极约束
//    Mat K = ( Mat_<double> ( 3,3 ) << 1472.09318784, 0, 668.02062312, 0, 1469.89378316, 362.86134410, 0, 0, 1 );
//    for ( DMatch m: matches )
//    {
//        Point2d pt1 = pixel2cam ( keypoints_1[ m.queryIdx ].pt, K );
//        Mat y1 = ( Mat_<double> ( 3,1 ) << pt1.x, pt1.y, 1 );
//        Point2d pt2 = pixel2cam ( keypoints_2[ m.trainIdx ].pt, K );
//        Mat y2 = ( Mat_<double> ( 3,1 ) << pt2.x, pt2.y, 1 );
//        Mat d = y2.t() * t_x * R * y1;
//        cout << "epipolar constraint = " << d << endl;
//    }


        img_temp=img_1_name;


        cout<<"t "<<t<<endl;
//        fout_c<<img_1_name << " "<<fixed<<t.at<double>(0) << " " <<t.at<double>(1)<< " " <<t.at<double>(2) <<endl;

        fout_c<<img_1_name << " "<<fixed<<t.at<double>(0) << " " <<t.at<double>(1)<< " " <<t.at<double>(2)<< " " \
                <<R.at<double> ( 0,0 )<<" "<<R.at<double> ( 0,1 )<<" "<<R.at<double> ( 0,2 )<<" "\
                        <<R.at<double> ( 1,0 )<< " " <<R.at<double> ( 1,1 )<< " " <<R.at<double> ( 1,2 )<< " " \
                                <<R.at<double> ( 2,0 )<< " " <<R.at<double> ( 2,1 )<< " " <<R.at<double> ( 2,2 )<<endl;

//        fout_c<<img_1_name << " "<<fixed<<t.at<double>(0) << " " <<t.at<double>(1)<< " " <<t.at<double>(2)<< " " \
//                <<R.at<double> ( 0,0 )<<" "<<R.at<double> ( 0,1 )<<" "<<R.at<double> ( 0,2 )<<" "\
//                        <<R.at<double> ( 1,0 )<< " " <<R.at<double> ( 1,1 )<< " " <<R.at<double> ( 1,2 )<< " " \
//                                <<R.at<double> ( 2,0 )<< " " <<R.at<double> ( 2,1 )<< " " <<R.at<double> ( 2,2 )<< " " \
//                                        << matches.size() <<" "<<E_empty<<" "<<RANSAC_outlier<<" "<<cam_front<<" "<<cam_front_raito<<endl;

    }
    myfile.close();
    fout_c.close();
    return 0;
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;

    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create(500);
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );




    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=100, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}


Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                    ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
                    ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
            );
}


//void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
//                            std::vector<KeyPoint> keypoints_2,
//                            std::vector< DMatch > matches,
//                            Mat& R, Mat& t )
//{
//    // 相机内参,TUM Freiburg2
////    Mat K = ( Mat_<double> ( 3,3 ) << 1472.09318784, 0, 668.02062312, 0, 1469.89378316, 362.86134410, 0, 0, 1 );
//    Mat K = ( Mat_<double> ( 3,3 ) <<1360.741577148438, 0, 658.3298488965738, 0, 1362.7548828125, 331.7633069283911, 0, 0, 1 );//dtl
//    //-- 把匹配点转换为vector<Point2f>的形式
//    vector<Point2f> points1;
//    vector<Point2f> points2;
//
//    for ( int i = 0; i < ( int ) matches.size(); i++ )
//    {
//        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
//        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
//    }
//
////    //-- 计算基础矩阵
////    Mat fundamental_matrix;
////    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
////    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;
//
//    //-- 计算本质矩阵
////    Point2d principal_point ( 668.02062312, 362.86134410); //相机光心, TUM dataset标定值
////    double focal_length =1472;			//相机焦距, TUM dataset标定值
//
//    Point2d principal_point ( 658.3298488965738,331.7633069283911); //相机光心, dtl标定值
//    double focal_length =1360.741577148438;			//相机焦距, dtl dataset标定值
////    Mat essential_matrix;
////    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
//
//    Mat mask;// https://blog.csdn.net/qq_26499769/article/details/51817254  //本质矩阵
//    Mat essential_matrix= findEssentialMat(points1, points2, focal_length, principal_point, RANSAC, 0.999, 1.0, mask);
//    if (essential_matrix.empty())
//        cout<<"essential_matrix "<<endl; //    if (E.empty()) return false;
//    double feasible_count = countNonZero(mask);
//    cout << (int)feasible_count << " -in- " << points1.size() << endl;
//    //对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
//    if (feasible_count <= 15 || (feasible_count /points1.size()) < 0.6)
//        cout<<"outlier "<<endl;//return false;
//
//    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;
//
////    //-- 计算单应矩阵
////    Mat homography_matrix;
////    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
////    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;
//
//    //-- 从本质矩阵中恢复旋转和平移信息.
////    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point);
//    //分解本征矩阵，获取相对变换
//
//    //同时位于两个相机前方的点的数量要足够大
//    int pass_count = recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point, mask);
//    if (((double)pass_count) / feasible_count < 0.7)
//        cout<<"outlier r t "<<endl;//return false;
//    cout<<"R is "<<endl<<R<<endl;
//    cout<<"t is "<<endl<<t<<endl;
//}







void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t )
{
    // 相机内参,dtl
    Mat K = ( Mat_<double> ( 3,3 ) <<1360.741577148438, 0, 658.3298488965738, 0, 1362.7548828125, 331.7633069283911, 0, 0, 1 );//dtl
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }


    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_RANSAC,5.0,0.999); //FM_RANSAC,CV_FM_8POINT
    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat  essential_matrix_f = K.t()*fundamental_matrix*K;
    cout<<"fundamental_matrix is "<<endl<< essential_matrix_f<<endl;
//    cout<<"Essential Matrix from Fundamental is "<<endl<< Essential_Fundamental_matrix<<endl;
    //-- 计算本质矩阵
    Point2d principal_point (658.3298488965738,331.7633069283911);	//相机光心, TUM dataset标定值
    double focal_length = 1360;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix_f, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;

}

void pose_estimation_2d2d_s ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t ,int& E_empty,int& RANSAC_outlier, int& cam_front,double& cam_front_raito)
{
    // 相机内参,TUM Freiburg2
//    Mat K = ( Mat_<double> ( 3,3 ) << 1472.09318784, 0, 668.02062312, 0, 1469.89378316, 364.68780905, 0, 0, 1 ); //sll
    Mat K = ( Mat_<double> ( 3,3 ) <<1360.741577148438, 0, 658.3298488965738, 0, 1362.7548828125, 331.7633069283911, 0, 0, 1 );//dtl
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;
    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

//    //-- 计算基础矩阵
//    Mat fundamental_matrix;
//    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
//    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
//    Point2d principal_point ( 668.02062312, 364.68780905); //相机光心, sll标定值
//    double focal_length =1472;			//相机焦距, sll标定值

    Point2d principal_point ( 658.3298488965738,331.7633069283911); //相机光心, dtl标定值
    double focal_length =1360.741577148438;			//相机焦距, dtl dataset标定值
//    Mat essential_matrix;
//    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );

    Mat mask;// https://blog.csdn.net/qq_26499769/article/details/51817254  //本质矩阵
//    Mat essential_matrix= findEssentialMat(points1, points2, focal_length, principal_point, RANSAC, 0.999, 1.0, mask);
    Mat essential_matrix= findEssentialMat(points1, points2, focal_length, principal_point,RANSAC,0.999,6.0,mask);//MEDS
    E_empty=0;
    if (essential_matrix.empty())
        E_empty=1;//cout<<"essential_matrix "<<endl; //    if (E.empty()) return false;
    double feasible_count = countNonZero(mask);
    cout << (int)feasible_count << " -in- " << points1.size() << endl;
    //对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
    RANSAC_outlier=0;
    if (feasible_count <= 15 || (feasible_count /points1.size()) < 0.6)
        RANSAC_outlier=1;//cout<<"outlier "<<endl;//return false;

    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

//    //-- 计算单应矩阵
//    Mat homography_matrix;
//    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
//    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
//    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point);
    //分解本征矩阵，获取相对变换

//    //同时位于两个相机前方的点的数量要足够大
//    int pass_count = recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point, mask);
//    cam_front=0;
//    if (((double)pass_count) / feasible_count < 0.7)
//        cam_front=1;
//    cam_front_raito=((double)pass_count) / feasible_count;

    //同时位于两个相机前方的点的数量要足够大
    int pass_count = recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point, mask);
    cam_front=0;
    if (((double)pass_count) / feasible_count < 0.7)
    {
        cam_front=1;

    }

    cam_front_raito=((double)pass_count) / feasible_count;

    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
}



void pose_estimation_2d2d_ss ( std::vector<KeyPoint> keypoints_1,
                              std::vector<KeyPoint> keypoints_2,
                              std::vector< DMatch > matches,
                              Mat& R1, Mat& R2, Mat& t ,int& E_empty,int& RANSAC_outlier, int& cam_front,double& cam_front_raito)
{
    // 相机内参,TUM Freiburg2
//    Mat K = ( Mat_<double> ( 3,3 ) << 1472.09318784, 0, 668.02062312, 0, 1469.89378316, 364.68780905, 0, 0, 1 ); //sll
    Mat K = ( Mat_<double> ( 3,3 ) <<1360.741577148438, 0, 658.3298488965738, 0, 1362.7548828125, 331.7633069283911, 0, 0, 1 );//dtl
    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;
    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

//    //-- 计算基础矩阵
//    Mat fundamental_matrix;
//    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
//    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
//    Point2d principal_point ( 668.02062312, 364.68780905); //相机光心, sll标定值
//    double focal_length =1472;			//相机焦距, sll标定值

    Point2d principal_point ( 658.3298488965738,331.7633069283911); //相机光心, dtl标定值
    double focal_length =1360.741577148438;			//相机焦距, dtl dataset标定值
//    Mat essential_matrix;
//    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );

    Mat mask;// https://blog.csdn.net/qq_26499769/article/details/51817254  //本质矩阵
    Mat essential_matrix= findEssentialMat(points1, points2, focal_length, principal_point, RANSAC, 0.999, 1.0, mask);
    E_empty=0;
    if (essential_matrix.empty())
        E_empty=1;//cout<<"essential_matrix "<<endl; //    if (E.empty()) return false;
    double feasible_count = countNonZero(mask);
    cout << (int)feasible_count << " -in- " << points1.size() << endl;
    //对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
    RANSAC_outlier=0;
    if (feasible_count <= 15 || (feasible_count /points1.size()) < 0.6)
        RANSAC_outlier=1;//cout<<"outlier "<<endl;//return false;

    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

//    //-- 计算单应矩阵
//    Mat homography_matrix;
//    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
//    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
//    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point);
    //分解本征矩阵，获取相对变换

//    //同时位于两个相机前方的点的数量要足够大
//    int pass_count = recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point, mask);
//    cam_front=0;
//    if (((double)pass_count) / feasible_count < 0.7)
//        cam_front=1;
//    cam_front_raito=((double)pass_count) / feasible_count;

    //同时位于两个相机前方的点的数量要足够大

    decomposeEssentialMat(essential_matrix, R1, R2,t );
    //todo
//    int pass_count = recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point, mask);
//    cam_front=0;
//    if (((double)pass_count) / feasible_count < 0.7)
//    {
//        cam_front=1;
//    }
//
//    cam_front_raito=((double)pass_count) / feasible_count;

    cout<<"R1 is "<<endl<<R1<<endl;
    cout<<"R2 is "<<endl<<R2<<endl;
    cout<<"t is "<<endl<<t<<endl;
}

