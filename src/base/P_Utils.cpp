#include "P_Utils.h"
#include "P_CoorTrans.h"
namespace Position
{
    //图像像素 相似度计算
    void PUtils::CheckPixelSimilarity(const Mat &img1, const Mat &img2)
    {
        assert(img1.size() == img2.size());

        int total = 0;
        for(int i = 0;i < img1.rows; ++i)
        {
            const uchar *pimg1 = img1.ptr<uchar>(i);
            const uchar *pimg2 = img2.ptr<uchar>(i);
            for(int j = 0; j < img1.cols; ++j)
            {
                if(pimg1[j] != pimg2[j])
                    ++total;
            }
        }
        PROMT_V("Image Similarity:",(1.0 - ((float)total / (img1.rows * img1.cols))) * 100 , "%");
    }

    //直方图 相似度计算
    void PUtils::CheckHistSimilarity(const Mat &img1, const Mat &img2)
    {
        assert(img1.size() == img2.size());
        Mat gimg1 = img1;
        Mat gimg2 = img2;
        if(img1.channels() > 1)
        {
            cvtColor(gimg1,gimg1,CV_RGB2GRAY);
        }
        if(img2.channels() > 1)
        {
            cvtColor(gimg2,gimg2,CV_RGB2GRAY);
        }

        int histsize = 256;
    	float range[] = { 0,256 };
        const float*histRanges = { range };
        Mat hist1,hist2;
    	calcHist(&gimg1, 1, 0, Mat(), hist1, 1, &histsize, &histRanges);
        calcHist(&gimg2, 1, 0, Mat(), hist2, 1, &histsize, &histRanges);
        assert(hist1.type() == hist2.type());
        float d = (1 - cv::compareHist(hist1,hist2,CV_COMP_BHATTACHARYYA)) * 100;
        PROMT_V("Hist Similarity ", d, "%" );
    //  putText(img, std::to_string(basebase), Point(50, 50), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2, CV_AA);
    }
    
    //绘制特征匹配
    Mat PUtils::DrawFeatureMatch(const Mat &img1, const Mat &img2,
                      const KeyPtVector &pt1s, const KeyPtVector &pt2s,
                      const MatchVector &matches,
                      const U8Vector &status /*= U8Vector()*/)
     {
        int w = img1.cols;
        int h = img1.rows;
        Mat keyimg1;
        Mat keyimg2;
        drawKeypoints(img1,pt1s,keyimg1,CV_RGB(0,0,255));
        drawKeypoints(img2,pt2s,keyimg2,CV_RGB(0,0,255));
        const int textpos = 50;
        putText(keyimg1,"keypoint size:" + std::to_string(pt1s.size()),cv::Point2f(textpos,textpos),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255,0,0), 2, CV_AA);
        putText(keyimg2,"keypoint size:" + std::to_string(pt2s.size()),cv::Point2f(textpos,textpos),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255,0,0), 2, CV_AA);
        
        Mat matchimg;
        hconcat(keyimg1, keyimg2, matchimg);
        Mat imkeycombine;
        hconcat(keyimg1,keyimg2,imkeycombine);
        int count = 0;
        for(int i = 0;i < matches.size();++i)
        {
            Point2f ptf1 =  pt1s[matches[i].queryIdx].pt;
            Point2f ptf2 =  pt2s[matches[i].trainIdx].pt + Point2f(w,0);
            const int thickness = 3;
            const int radius    = 2;
            if(status.empty() || status[i])
            {//right match
                circle(matchimg,ptf1,radius,CV_RGB(0,255,0), thickness);
                circle(matchimg,ptf2,radius,CV_RGB(0,255,0), thickness);
                line(matchimg,ptf1,ptf2,CV_RGB(0,255,0),thickness - 1);
                ++count;
            }
            else
            {
                circle(matchimg,ptf1,radius,CV_RGB(255,0,0), thickness);
                circle(matchimg,ptf2,radius,CV_RGB(255,0,0), thickness);
                line(matchimg,ptf1,ptf2,CV_RGB(255,0,0),thickness - 1);
            }
        }
        putText(matchimg,"total match:" + std::to_string(matches.size()),cv::Point2f(textpos,textpos * 2),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255,0,0), 2, CV_AA);
        putText(matchimg,"good  match:" + std::to_string(count) + " | " + std::to_string(100 * (count / (float)matches.size())) + "%",cv::Point2f(textpos,textpos * 3),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255,0,0), 2, CV_AA);

        Mat result;
        vconcat(imkeycombine,matchimg,result);
        return result;
    }

    //ax + by + c = 0
    void PUtils::DrawEpiLine(MATTYPE a, MATTYPE b, MATTYPE c, const Point2f &pt, Mat &img)
    {
        if(!img.empty())
        {
            Point2f bg;
            bg.x = 0;
            bg.y = - c/b ;


            Point2f ed;
            ed.x = img.cols;
            ed.y = -(c + a * ed.x) / b;

            line(img,bg,ed,CV_RGB(0,255,0));
            const int thickness = 2;
            circle(img,pt,thickness,CV_RGB(255,0,0),thickness);
            
            float distance = -1;
            if(pt.x > 0)
            {
                Point2f foot = ComputeFootPoint(a,b,c,pt);
                line(img,pt,foot,CV_RGB(255,255,0));
                circle(img,foot,thickness,CV_RGB(255,255,0),thickness);
                distance = cv::norm(foot-pt);
            }   
            putText(img,"Read  : Point"     ,Point2f(50,50) ,CV_FONT_HERSHEY_COMPLEX,2,CV_RGB(255,0,0)  ,2,CV_AA);
            putText(img,"Yellow: FootPt"    ,Point2f(50,150),CV_FONT_HERSHEY_COMPLEX,2,CV_RGB(255,255,0),2,CV_AA); 
            putText(img,"Green : Epiline"   ,Point2f(50,250),CV_FONT_HERSHEY_COMPLEX,2,CV_RGB(0,255,0)  ,2,CV_AA);
            // putText(img,string("Distance: ") + std::to_string(distance) ,Point2f(50,350),CV_FONT_HERSHEY_COMPLEX,2,CV_RGB(0,0,255)  ,2,CV_AA);
        }
    }
     //! 固定窗口大小显示图片
    void PUtils::ShowImage(const string &name, Mat &img)
    {
        cv::namedWindow(name,CV_WINDOW_NORMAL);
        cv::resizeWindow(name,Size(1080,720));
        cv::moveWindow(name,0,0);
        setWindowProperty(name,CV_WND_PROP_FULLSCREEN,CV_WINDOW_NORMAL);
        imshow(name,img);
    }

     //! 图片并排显示
    void PUtils::ShowHImages(const string &name, cv::InputArrayOfArrays imgs)
    {
        std::vector<cv::Mat> mats;
        imgs.getMatVector(mats);
        if(mats.size() != 2)
            return;
        if(mats[0].size() != mats[1].size())
        {
            cv::resize(mats[1],mats[1],mats[0].size());
        }
        cv::Mat img;
        cv::hconcat(mats,img);
        ShowImage(name,img);
    }

    //！ 图片并列显示
    void PUtils::ShowVImages(const string &name, cv::InputArrayOfArrays imgs)
    {
        std::vector<cv::Mat> mats;
        imgs.getMatVector(mats);
        if(mats.size() != 2)
            return;
        if(mats[0].size() != mats[1].size())
        {
            cv::resize(mats[1],mats[1],mats[0].size());
        }
        cv::Mat img;
        cv::vconcat(mats,img);
        ShowImage(name,img);
    }

    //直方图均衡
    void PUtils::ImageHistEqualized(const Mat &img, Mat &outimg)
    {
        if(img.empty())
            return;

        if(img.channels() > 1)
        {
            vector<Mat> channels;
            Mat blue,green,red;
            //拆分通道 默认rgb格式
            cv::split(img,channels);
            red   = channels.at(0);
            green = channels.at(1);
            blue  = channels.at(2);
            cv::equalizeHist(red  , red)  ;
            cv::equalizeHist(green, green);
            cv::equalizeHist(blue , blue) ;

            cv::merge(channels,outimg);
        }
        else
        {//单通道
            cv::equalizeHist(img,outimg);
        }
    }
    //绘制直方图
    void PUtils::HistDraw(const Mat &img)
    {
        if(img.empty())
        {
            return ;
        }

        if(img.channels() < 2)
        {
            //步骤二：计算直方图
    	    int histsize = 256;
    	    float range[] = { 0,256 };
            const float*histRanges = { range };
            Mat hist;
    	    calcHist(&img, 1, 0, Mat(), hist, 1, &histsize, &histRanges);
            //归一化
            int hist_h = 400;//直方图的图像的高
            int hist_w = 512;////直方图的图像的宽
            int bin_w = hist_w / histsize;//直方图的等级
            Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0, 0, 0));//绘制直方图显示的图像
            normalize(hist, hist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
            //步骤三：绘制直方图（render histogram chart）
            for (int i = 1; i < histsize; i++)
            {
                //绘制直方图
                line(histImage, Point((i - 1)*bin_w, hist_h - cvRound(hist.at<float>(i - 1))),
                    Point((i)*bin_w, hist_h - cvRound(hist.at<float>(i))), Scalar(255, 0, 0), 2, CV_AA);
            }
            PROMTD_S("Hist image display.");
            imshow("hist",histImage);
            waitKey(0);
        }
        else
        {
            vector<Mat> channels;
            cv::split(img,channels);
             //步骤二：计算直方图
    	    int histsize = 256;
    	    float range[] = { 0,256 };
            const float*histRanges = { range };
            Mat rhist,ghist,bhist;
    	    calcHist(&channels[0], 1, 0, Mat(), rhist, 1, &histsize, &histRanges);
            calcHist(&channels[1], 1, 0, Mat(), ghist, 1, &histsize, &histRanges);
            calcHist(&channels[2], 1, 0, Mat(), bhist, 1, &histsize, &histRanges);

            //归一化
            int hist_h = 400;//直方图的图像的高
            int hist_w = 512;////直方图的图像的宽
            int bin_w = hist_w / histsize;//直方图的等级
            Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0, 0, 0));//绘制直方图显示的图像
            normalize(rhist, rhist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
            normalize(ghist, ghist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
            normalize(bhist, bhist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
            //步骤三：绘制直方图（render histogram chart）
            for (int i = 1; i < histsize; i++)
            {
                //绘制r直方图
                line(histImage, Point((i - 1 )* bin_w, hist_h - cvRound(rhist.at<float>(i - 1))),
                    Point((i)*bin_w, hist_h - cvRound(rhist.at<float>(i))), CV_RGB(255,0,0), 2, CV_AA);
                //绘制g直方图
                line(histImage, Point((i - 1) * bin_w, hist_h - cvRound(ghist.at<float>(i - 1))),
                    Point((i)*bin_w, hist_h - cvRound(ghist.at<float>(i))), CV_RGB(0,255,0), 2, CV_AA);
                //绘制b直方图
                line(histImage, Point((i - 1) * bin_w, hist_h - cvRound(bhist.at<float>(i - 1))),
                    Point((i)*bin_w, hist_h - cvRound(bhist.at<float>(i))), CV_RGB(0,0,255), 2, CV_AA);
            }
            PROMTD_S("Hist image display.");
            imshow("hist",histImage);
            waitKey(0);
        }
    }

    static inline double haverSin(double x)
    {
        double v = sin(x / 2.0);
        return v * v;
    }

    /* get distance from two frames
     */
    double PUtils::ComputeDistBetBLH(const BLHCoordinate &blh1, const BLHCoordinate &blh2)
    {
        double radlon1 = D2R(blh1.lon);
        double radlat1 = D2R(blh1.lat);
        double radlon2 = D2R(blh2.lon);
        double radlat2 = D2R(blh2.lat);
        
        double a = fabs(radlat1 - radlat2);
        double b = fabs(radlon1 - radlon2);
        
        double h = haverSin(b) + cos(blh1.lat)*cos(blh2.lat)*haverSin(a);
        double distance = 2 * WGS84Datum.r_max * asin(sqrt(h));
        return  distance;
    }

    /* 从位姿获取R和t
     *
     */
    void PUtils::ComputeRtFromPose(const PoseData &predata,
                                   const PoseData &curdata,
                                   const Mat cam2imuR,
                                   const Mat cam2imuT,
                                   Mat &R, Mat &t)
    {
        const BLHCoordinate &blht1 = predata.pos;
        const BLHCoordinate &blht2 = curdata.pos;

        //计算imu到enu 转换矩阵
        cv::Mat Rimu2Enu1 = PCoorTrans::IMU_to_ENU(-predata._yaw, predata._pitch, predata._roll);
        cv::Mat Rimu2Enu2 = PCoorTrans::IMU_to_ENU(-curdata._yaw, curdata._pitch, curdata._roll);

        //计算xyz转到enu 转换矩阵
        cv::Mat XYZ2Enu1 = PCoorTrans::XYZ_to_ENU(blht1.lat, blht1.lon);
        cv::Mat XYZ2Enu2 = PCoorTrans::XYZ_to_ENU(blht2.lat, blht2.lon);

        //imu到 xyz转换矩阵
        cv::Mat Rimu2xyzt1 = XYZ2Enu1.t() * Rimu2Enu1;
        cv::Mat Rimu2xyzt2 = XYZ2Enu2.t() * Rimu2Enu2;

        Point3d xyzt1;
        Point3d xyzt2;
        //获取xyz坐标
        xyzt1 = PCoorTrans::BLH_to_XYZ(blht1);
        xyzt2 = PCoorTrans::BLH_to_XYZ(blht2);
        cv::Mat pt1 = (cv::Mat_<double>(3, 1) << xyzt1.x, xyzt1.y, xyzt1.z);
        cv::Mat pt2 = (cv::Mat_<double>(3, 1) << xyzt2.x, xyzt2.y, xyzt2.z);

        //相对旋转矩阵
        R = cam2imuR.t() * Rimu2xyzt2.t() * Rimu2xyzt1 * cam2imuR;

        //计算cur相机在xyz坐标系中坐标
        cv::Mat curCamPos = Rimu2xyzt2 * cam2imuT + pt2;
        //计算以pre为原点建立的imu坐标系,pt2相机的位置
        cv::Mat imut1Pcam = Rimu2xyzt1.t() * (curCamPos - pt1);
        //计算以pre为原点建立的cam坐标系,pt2相机的位置
        cv::Mat camt1Pcam = cam2imuR.t() * imut1Pcam - cam2imuR.t() * cam2imuT;

        //以cam2的位置 反推t  这里r * cam2 只是计算方向
        t = -R * camt1Pcam; 
    }

    /* 通过帧间R、t推算绝对坐标
     *
     */
    void PUtils::ComputePoseFromRT(const PoseData &origin,
                                   const Mat &R, const Mat &t,
                                   const Mat &cam2imuR,
                                   const Mat &cam2imuT,
                                   BLHCoordinate &blh,
                                   const PoseData &realdst)
    {
        const BLHCoordinate &ogngps = origin.pos;

        const Point3d xyz = PCoorTrans::BLH_to_XYZ(ogngps);

        const Mat m_xyz = (Mat_<double>(3, 1) << xyz.x, xyz.y, xyz.z);

        Mat imu2enu = PCoorTrans::IMU_to_ENU(-origin._yaw, origin._pitch, origin._roll);

        const Mat xyz2enu = PCoorTrans::XYZ_to_ENU(ogngps.lat, ogngps.lon);

        //imu坐标系->xyz坐标系旋转矩阵
        Mat imu2xyz = xyz2enu.inv() * imu2enu;
        //计算pre相机坐标系 cur相机位置
        Mat dstcampt = -R.inv() * t;
        //相机坐标系->imu坐标系
        Mat dstimupt = cam2imuR * dstcampt + cam2imuT;
        //imu坐标系->xyz坐标系
        Mat dstxyzpt = imu2xyz * dstimupt + m_xyz;
        //cam位置转到imu位置
        dstxyzpt = dstxyzpt - imu2xyz * cam2imuT;

        Point3d dstxyz(dstxyzpt.at<double>(0, 0),
                       dstxyzpt.at<double>(1, 0),
                       dstxyzpt.at<double>(2, 0));

        blh = PCoorTrans::XYZ_to_BLH(dstxyz);

        if (realdst._t > 0)
        {
            Point3d orngauss = PCoorTrans::BLH_to_GaussPrj(realdst.pos);
            Point3d dstgauss = PCoorTrans::BLH_to_GaussPrj(blh);

            PROMTD_V("dif",(orngauss.x - dstgauss.x),
                           (orngauss.y - dstgauss.y),
                           (orngauss.z - dstgauss.z));
        }
    }
}