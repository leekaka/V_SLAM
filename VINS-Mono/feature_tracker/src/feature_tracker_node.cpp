#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];  // 所有�?�理和数�?类型都通过这个类的数组处理

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

/*核心函数*/
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag)  // 首帧�?记时间戳，不处理，直�?return
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }



    // detect unstable camera stream
    //如果前后两帧图像时延超过1s || 甚至后一帧时间戳比前一帧还�? ==> 说明图像流不稳定，则发送restart_flag到topics�?，重�?算法
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;

        std_msgs::Bool restart_flag;  // 发布的重�?消息
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();



    // frequency control

    // FREQ在yml文件�?设置：当yml文件�?设置�?0，在readParameters(n)函数�?，重新�?�置为FREQ  �?100�?
    // 此时一般pushlish的�?�率等于图像原帧�?
    // �?有图像帧率小于FREQ的时候才会发布图像信�?，但�?所有原帧率的图像都参与了�?�算，可以实现后续node的降�?

    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;

        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    /*
        后面针�?�图像msg的格式进行了要求�?
        �?识别灰度图格�?8UC1和MONO8�?
        并最终将编码都转�?为MONO8格式�?
        但其实MONO8格式�?8UC1�?一样的�?
        没做什么变�?。最后将图像经过cv_bridge库转�?为opencv的常用格式�?

        ros图像msg类型，转�?为cv类型，图像指针为 cv_bridge::CvImageConstPtr ptr
    */
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;

    /*
        读取到图像并�?成cv格式后，正式开始�?�理�?
    */
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)  //因为�?单目，所�? trackerData �?有一维，也就�?所有的处理都通过 �? FeatureTracker实现，具体通过该类的一�?函数readImage实现�?
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    /*
        遍历ids，将新添加的特征点编号，每个特征的编号id�?一无二�?
        前后两帧�?相同特征的编号是一样的，而且相同id的特征在图像�?一定是连续出现的，
        因为不连�?的话，当新出现的时候会给他一�?新的id�?
    */

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

    /**
     *  track_cnt反映了�?�应特征连续在几幅图像中出现�?
        �?有连�?两�?�以上都出现在图像里的特征�?�publish
        每个特征的id号是�?一无二的，所有图像中通过光流!连续!检测到的特征的编号�?一样的。�?�果该特征中途丢失，再回来�?�测到，id就不一样了
        此�?�，因为goodFeaturesToTrack函数，有�?能存在同一�?特征在同一幅图像中�?标了两�??id，但不影响前后帧上的同特征标号相同这一特�?
        校�?�畸变后的[(u-cx)/fx,(v-cy)/fy,1], id_of_point, u_of_point, v_of_point,(校�?�前) velocity_x_of_point都是向量，是一整幅图的信息
     **/

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);    //校�?�畸变后的[(u-cx)/fx,(v-cy)/fy,1]
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);   //特征的id
        feature_points->channels.push_back(u_of_point);   //(u-cx)/fx
        feature_points->channels.push_back(v_of_point);   //(v-cy)/fy
        feature_points->channels.push_back(velocity_x_of_point);  //光流x方向速度除以焦距x �?(deltu/fx)/dt，校正后的像素�?�算出的
        feature_points->channels.push_back(velocity_y_of_point);  //光流y方向速度除以焦距y �?(deltv/fy)/dt，校正后的像素�?�算出的

        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    readParameters(n);  //读取参数

    /*
        每个node都重新定义了readParameter函数�?
        都有新的parameters.cpp�?.hpp，因为是属于不同的进程，读取的具体内容都有不同�?
    */

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);  //CAM_NAMES[i]是config_file,也就是yml文件//�?持四种模型：kannala_brandt，mei，scaramuzza，pinhole
    /*
        不同的camera类型对应不同的模板，然后对应到不同的文件�?
        例�?�pinhole类型对应了PinholeCamera.cc和PinholeCamera.h，是PinholeCamera类�?
    */

    if(FISHEYE)  //如果是fisheye，�?��?�取相应的mask文件，�?�果不是，就不执行，mask为全255�?
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    //订阅了图像信息IMAGE_TOPIC, 回调函数img_callback�?干了所有事�?
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);


    //初�?�化需要发布的三�?�消�?
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */

    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?