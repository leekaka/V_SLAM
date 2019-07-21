#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)  // 根据status的状态，重置一下vector<>V,只保留status可以的
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}


/*
    1、readImage函数          将图像的特征点以及光流速度都计算出来，存储在trackerData[i]的变量，
    2、校正后的                特征点存储在cur_un_pts（本帧，包含新添加的特征）和 pre_un_pts（上帧，和本帧添加新点之前的特征点已经对齐）中，
                  光流速度 pts_velocity 、cur_pts和pre_pts  是未经过校正的像素位置
    3、cur_un_pts   和pre_un_pts   并不是简单的像素位置，而是[(u-cx)/fx,[(v-cy)/fy];   pts_velocity也不是单纯的像素速度，而是（像素速度/fx），即 [(deltu/fx)/dt,(deltv/fy)/dt]
*/
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)  //读取图像和时间戳
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    /*首先如果EQUALIZE为1，先对图像亮度进行了调整*/
    if (EQUALIZE)  
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    
    /*如果是首次计算，那么先跳过光流计算函数，跳到  goodFeaturesToTrack */
    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        /*送入光流计算的图像和点都是未做畸变校正的点*/
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);  // 新计算出来的特征点 forw_pts

        for (int i = 0; i < int(forw_pts.size()); i++)  //inBorder 保证新计算出来的有效特征forw_pts 位于图像内, 将有效的，但是位置正好在边界上的特征点去掉
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        /*除了第一次通过 goodFeaturesToTrack 求得角点外，
        其他都是通过  calcOpticalFlowPyrLK  求得的，
        而且每次都用  status  把相同的部分保留下来*/

        /*
            所以最后prev_pts，cur_pts，forw_pts都只有相同的角点被保留下来了，最后角点数目会越来越少，如果不做处理，最后size一定会变成0
            为了保证有效的角点数目，因此在后面的程序中检查了当前有效的角点数forw_pts.size()，如果小于MAX_CNT的话，就调用goodFeaturesToTrack，多算出MAX_CNT - forw_pts.size()个角点，
            添加到forw_pts后面，保证forw_pts的特征数量一致保持在MAX_CNT
            但是这里有个问题，goodFeaturesToTrack新计算出来的点有可能在forw_pts已经存在了，也可能不存在
            虽然经过了补充，prev_pts的特征数目会比cur_pts和forw_pts少，但是前面的特征还是一一对应的，可以用status来处理
            注意：经过status处理后！那么，prev_pts  一直保存的是prev_pts，cur_pts, forw_pts都包含的特征，而且一直在被更新

            cur_pts 保存的是cur_pts,forw_pts都包含的特征，
            而且因为经过goodFeaturesToTrack的添加，
            因此会比  prev_pts 个数多。
            在这里，forw_pts还没被添加新点，
            因此此时cur_pts 和 forw_pts还是一样的点数
        */

        reduceVector(prev_pts, status);   //前前帧：第一次的时候prev_pts是空的
        reduceVector(cur_pts, status);    //前帧：第一次的时候cur_pts在经过status处理前是通过goodFeaturesToTrack求得MAX_CNT个点，
        
        // 后面也通过goodFeaturesToTrack补充了新的角点，使他一直保持MAX_CNT个

        reduceVector(forw_pts, status);   //当前帧：forw_pts 在经过status处理前，是通过 calcOpticalFlowPyrLK 求得的
        reduceVector(ids, status);        //前帧，ids 第一次的时候，在经过status处理前，个数 等于 前帧的个数，而且值都是-1
        reduceVector(cur_un_pts, status); //前帧，cur_un_pts在经过status处理前，是cur_pts经过校正后的特征点
        reduceVector(track_cnt, status);  //前帧，track_cnt第一次的时候，在经过status处理前，个数等于前帧的个数，而且值都是1

        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)  //vector<int> track_cnt，这里n++会导致track_cnt向量中的元素递增，track_cnt反映了该特征  连续在几幅图像中出现过
        n++;

    if (PUB_THIS_FRAME)  // 这意味着如果降频，原图像参加了cv光流的计算，但是没有去除相应的噪点和goodfeature更新角点的过程
    {
        rejectWithF();  //利用ransac 剔除prev_pts，cur_pts，forw_pts，ids，cur_un_pts，track_cnt 中一些不可信的点 //剔除误匹配，随机抽样一致性

        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();      //如果有鱼眼mask，去掉mask黑色部分的特征
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());

        /**
         * 
         * 当光流计算出来的相同的角点少于MAX_CNT后，就重新调用goodFeaturesToTrack更新角点
         * 
         * */
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
            /*
                forw_img等图像都没经过畸变校正，直到求出特征点后才再后面做畸变校正并存入到另一个向量 cur_un_pts 中
                n_pts 是返回的角点坐标
                MAX_CNT - forw_pts.size()是返回的最大角点数目；0.01是角点的品质因子
                MIN_DIST 通过yml读取，目前设置为30。初选角点，如果在它周围MIN_DIST范围内出现比他更强的角点，则删除该角点。
                mask：指定感兴趣的区域，如不需要在整幅图上寻找感兴趣的角点，可以用mask设置ROI区域。
                例如原需要用mask的fisheye就不计算整个区域
            */
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());


        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();  // n_pts

        /*
        通过上面我们首次计算出MAX_CNT个角点，然后addPoints();

        for (auto &p : n_pts)
        {
            forw_pts.push_back(p);
            ids.push_back(-1);
            track_cnt.push_back(1);
        }
        给 forw_pts 添加角点坐标，
        给ids添加  等于   新增角点 个数的 -1，
        给 track_cnt 添加等于 角点个数的 1
        只有函数 updateID 才会更新 ids，使他不为-1，
        而 updateID 才feature_tracker_node.cpp 中才调用
        */

        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    /*在函数的最后，cur_pts 更新为当前帧，prev_pts、prev_un_pts 更新为上一帧（首次的时候为NULL）*/
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;

    undistortedPoints();   // 将cur_pts 做畸变校正，结果存入 cur_un_pts

    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status); //通过求解基础矩阵时的参数RANSAC,利用status删除误差匹配点
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)  //
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
