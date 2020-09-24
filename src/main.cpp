/*
환경 : zed_mini
homography matrix를 이용한 상대 위치 출력 프로그램입니다.
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <algorithm>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

using namespace cv;
using namespace std;


double standard_roll,standard_pitch;

Mat image_bird1;

Mat K; // intrinsic parameter

double altitude = 0.71; // zed_mini의 높이를 설정해주세요 (m 단위) -> homography의 scale추정에 쓰입니다.


struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);

Mat rpToHomography(float roll, float pitch, Mat K);

void makeBirdEyeView(Mat& in,Mat& out,float roll,float pitch, Mat K);

void image2_cb(const sensor_msgs::Image::ConstPtr& image,const sensor_msgs::Imu::ConstPtr& rpy);

int findRelativePose(Mat& in1, Mat& in2, Mat K);


int main(int argc, char** argv){
    //현재 image_in2을 기준으로 homography를 구해 position과 orientation을 구합니다!//

    //현재 상태
    //image_in1 : 정적이미지
    //image_in2 : zed_mini로부터 실시간으로 들어오는 영상

    getMyPackagePath();


    Mat image_in1;
    image_in1 = imread("home/kwon/catkin_ws/image/image.png",IMREAD_COLOR);

    //intrinsic parameter 계산 //
    float f = 100, w = image_in1.size().width, h = image_in1.size().height;

    K = (Mat_<double>(3, 3) <<
      f, 0, w/2,
      0, f, h/2,
      0, 0,   1);

    // bird eye view 만드는 부분//
    makeBirdEyeView(image_in1,image_bird1,0,0,K);


    //--------------------------------------------------------------//

    ros::init(argc,argv,"homography_test");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh,"/zedm/zed_node/rgb/image_rect_color",10);
    message_filters::Subscriber<sensor_msgs::Imu> rpy2_sub(nh,"/zedm/zed_node/imu/data",10);
  //  message_filters::Subscriber<geometry_msgs::PoseStamped> pose1_sub(nh,"/mavros/local_position/pose",10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(500), image2_sub, rpy2_sub);
    sync.registerCallback(boost::bind(image2_cb,_1,_2));


    ros::Rate rate(20.0);

    while(nh.ok()){

      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp)*(180/M_PI);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp)*(180/M_PI); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp)*(180/M_PI);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp)*(180/M_PI);

    return angles;
}


void image2_cb(const sensor_msgs::Image::ConstPtr& image,const sensor_msgs::Imu::ConstPtr& rpy){



  Quaternion q;
  q.x = rpy->orientation.x;
  q.y = rpy->orientation.y;
  q.z = rpy->orientation.z;
  q.w = rpy->orientation.w;


  EulerAngles ea = ToEulerAngles(q);


  if(standard_roll == NULL){
    standard_roll = ea.roll;
    standard_pitch = ea.pitch;
  }else{
    ea.roll -= standard_roll;
    ea.pitch -= standard_pitch;
  }


  cv_bridge::CvImagePtr input = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);


  Mat image_in2 = input->image;
  Mat image_bird2;


  if(image_in2.empty()){
    cout<< "empty" <<endl;
  }else{
    makeBirdEyeView(image_in2,image_bird2,0,0,K);
    findRelativePose(image_bird1,image_bird2,K);
  }

  cout<< "roll : " << ea.roll << ", " << "pitch : "<< ea.pitch << ", yaw : " << ea.yaw << endl;




}

void makeBirdEyeView(Mat& in,Mat& out,float roll,float pitch, Mat K){
  Mat H1 = rpToHomography(roll,pitch,K);
  warpPerspective(in,out,H1,in.size());
}

Mat rpToHomography(float roll, float pitch, Mat K){ // angular 값을 받음
  //드론에서의 roll pitch를 이미지의 roll pitch로 전환
  float r = pitch;
  float p = roll;
  //라디안으로 변경//
  r = r*M_PI/180.0;
  p = p*M_PI/180.0;

  Mat roll_m =(Mat_<double>(3,3) <<
  1,    0,       0,
  0,cos(r),-sin(r),
  0,sin(r),cos(r));

  Mat pitch_m =(Mat_<double>(3,3) <<
  cos(p),   0,  sin(p),
      0,        1,      0,
  -sin(p),  0,  cos(p));

  Mat r_rot = K*roll_m*pitch_m*K.inv();

  /*
  Mat1f r_2D = (cv::Mat1f(3,3) <<
  r_rot(1,1), r_rot(1,2),0,
  r_rot(2,1), r_rot(2,2),0,
  0,  0,  1);
*/
  return r_rot;
}

int findRelativePose(Mat& in1, Mat& in2, Mat K){
  //특징점 매칭//
  vector<KeyPoint> keypoints_1,keypoints_2;
  Mat descriptors_1, descriptors_2;
  Ptr<ORB>orbF = ORB::create(1000);
  orbF->detectAndCompute(in1,noArray(), keypoints_1,descriptors_1);
  orbF->detectAndCompute(in2,noArray(),keypoints_2,descriptors_2);



  if(descriptors_1.empty() || descriptors_2.empty() || keypoints_1.empty() || keypoints_2.empty()){
    return 0;
  }

  BFMatcher matcher(NORM_HAMMING);
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches);

  sort(matches.begin(), matches.end(), [](DMatch a, DMatch b) { return a.distance < b.distance; });
  Mat out;
  std::vector< DMatch > matches_sorted;

  int t_n = 30;
  if(matches.size() >= t_n){ // 최대 t_n개의 선만 표시
  matches_sorted.assign(matches.begin(),matches.begin()+t_n);
  }else{
    matches_sorted.assign(matches.begin(),matches.end());
  }

  drawMatches(in1, keypoints_1, in2, keypoints_2, matches_sorted ,out);
  imshow("Original", out);

  waitKey(1);
  //--------------------------------------------------------------/
  //homography 적용
  DMatch t;
  vector<Point2f> orig,rot;
  //quertyIdx : original 이미지의 좌표
  //trainIdx : original 이미지 좌표에 매칭된 rot 이미지 좌표



  for(int i=0; i<matches.size(); i++){
    t = matches[i];
    orig.push_back(keypoints_1[t.queryIdx].pt);
    rot.push_back(keypoints_2[t.trainIdx].pt);
  }



  Mat homography = findHomography(orig,rot,RANSAC,5.0);



  if(homography.empty()){
    return 0;
  }
  std::vector<cv::Mat> Rs, Ts;
  decomposeHomographyMat(homography,K,Rs,Ts,cv::noArray());


  //상대 orientation 출력x
  std::cout << "rvec = " << std::endl;
  for (auto R_ : Rs) {
    cv::Mat1d rvec;
    cv::Rodrigues(R_, rvec);
    rvec =  rvec*180/CV_PI;
    printf("x: %.3lf ,y: %.3lf ,z: %.3lf \n",rvec.at<double>(0,0),rvec.at<double>(0,1),rvec.at<double>(0,2));
  }


  std::cout << std::endl;

  std::cout << "t = " << std::endl;
  for (auto t_ : Ts) {
    double scale = altitude/3.0;
    double x = scale*t_.at<double>(0,0);
    double y = scale*t_.at<double>(0,1);
    double z = t_.at<double>(0,2);

    printf("x: %.3lf m ,\t y: %.3lf m ,\t z: %.3lf m \n",x,y,z);
  }

  return 1;
}
