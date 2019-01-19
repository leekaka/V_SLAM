#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main ( int argc, char** argv)
{
    if (argc != 3)
    {
	cout<<"usage:feature_extraction img1 img2"<<endl;
	return 0;
    }

    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);

    //初始化
    vector<KeyPoint>keypoints_1,keypoints_2;
    Mat descriptors_1,descriptors_2;

    //iPtr<FeatureDetector>detector = ORB::create();
    //Ptr<ORB>orb = ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create( "BruteForce-Hamming" );

    // 检测
    detector->detect( img_1,keypoints_1 );
    detector->detect( img_2,keypoints_2 );
     
    //orb->detect(img_1,keypoints_1);
    cout<<"KP1="<<keypoints_1.size()<<endl;
    cout<<"KP2="<<keypoints_2.size()<<endl;

    // 根据角点位置计算 BRIEF 描述子

    descriptor->compute(img_1, keypoints_1,descriptors_1);
    descriptor->compute( img_2,keypoints_2,descriptors_2);

    Mat outimg1;
    drawKeypoints(img_1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("1.png features",outimg1);

    //--对两幅图像中的BRIEF描述子进行匹配
    vector<DMatch> matches;
    matcher->match ( descriptors_1,descriptors_2,matches);

    // -- 筛选

    double min_dist=10000, max_dist=0;

    for (int i=0;i<descriptors_1.rows;i++)   // 找到最大和最小距离
    {
	double dist = matches[i].distance;
	if ( dist < min_dist) min_dist = dist;
	if ( dist > max_dist) max_dist = dist;
    }

    cout<<"max_dist="<<max_dist<<endl;
    cout<<"min_dist="<<min_dist<<endl;

    std::vector< DMatch > good_matches;
    for ( int i=0;i<descriptors_1.rows;i++)
    {
	if ( matches[i].distance <=max (2*min_dist,10.0))
	{
	    good_matches.push_back( matches[i]);
	}
    }

    // 绘图
    Mat img_match;
    Mat img_goodmatch;
    drawMatches (img_1,keypoints_1, img_2, keypoints_2,matches,img_match);
    drawMatches (img_1,keypoints_1,img_2,keypoints_2, good_matches,img_goodmatch);

    imshow("all= ",img_match);
    imshow("good=",img_goodmatch);


    





    waitKey(0);

    return 0;
}
