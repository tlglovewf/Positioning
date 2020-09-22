#include "test.h"
#include "P_Utils.h"

cv::Mat do_canny(cv::Mat frame)
{
    cv::Mat gray;
    cv::cvtColor(frame, gray, COLOR_BGR2GRAY);
    cv::GaussianBlur(frame,frame,Size(1,1),0);
    cv::Mat cny;
    cv::Canny(frame,cny,400,350);

    return cny;
}

TESTBEGIN()

    //cv::Mat img = cv::imread("/media/tu/Work/GitHub/TwoFrameSO/data/inputim/0-006437-717-0007823.jpg");
    cv::Mat img = cv::imread("/media/tu/Work/Datas/test/222.jpg");
    
    cv::Rect rect;
    rect.x = 0;
    rect.y = img.rows / 3;
    rect.width = img.cols;
    rect.height = img.rows - rect.y;
    cv::Mat rot = img(rect);
    cv::Mat cny = do_canny(rot);

#if 1
    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(cny, lines, 1, CV_PI / 180.0, 80, 30, 50);
    cv::Mat mask(cny.size(),CV_8U,cv::Scalar(255)); 

    const Rect roi(0 ,cny.rows * 0.66 ,cny.cols ,cny.rows * 0.33);
    // const Rect roi(0 ,0 ,cny.cols ,cny.rows * 0.66);
    mask(roi).setTo(0);
    for(size_t i = 0; i < lines.size(); ++i)
    {
        Vec4i l = lines[i];
        cv::Point2f pt1(l[0],l[1]);
        cv::Point2f pt2(l[2],l[3]);

        cv::Mat tempLine = cv::Mat(cny.size(),CV_8U,cv::Scalar(0));

        cv::line(tempLine,pt1, pt2,cv::Scalar(255),3);
        
        cv::bitwise_and(cny, tempLine, tempLine);   
        
        tempLine(roi).setTo(0);

        std::vector<cv::Point> points;
        for(int y = 0; y < tempLine.rows; ++y)
        {
            uchar *rowPtr = tempLine.ptr<uchar>(y);
            for( int x = 0; x < tempLine.cols; ++x)
            {
                if( rowPtr[x] )
                {
                    points.emplace_back(cv::Point(x, y));
                }
            }
        }

        if(points.empty())
        {
            // PROMT_S("line points empty!!!");
            continue;
        }
        cv::Vec4f fline;
        cv::fitLine(points,fline,CV_DIST_HUBER,0, 0.01, 0.01);

        double cos_theta = fline[0];
        double sin_theta = fline[1];
        double x0 = fline[2], y0 = fline[3];
        const float v = 0.4;
        if(sin_theta < v && sin_theta > -v )
        {
            continue;
        }

        double k = sin_theta / cos_theta;

        double b = y0 - k * x0;

        double x = 0;
        double y = k * x + b;

        x0 = rot.cols;
        y0 = k * x0 + b;

        cv::line(rot,cv::Point(x0, y0),cv::Point(x,y),Scalar(255,0,0));
    }

#else 
    std::vector<cv::Vec2f> lines;

    cv::HoughLines(cny,lines,1, CV_PI / 180.0,50,0,0);
    cout << "line size " << lines.size() << endl;
    const int alpha = 1000;//alpha取得充分大，保证画出贯穿整个图片的直线
    for(size_t i = 0; i < lines.size(); ++i)
    {
        float rho = lines[i][0], theta = lines[i][1];
        cout << "radius : " << rho << endl;
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + alpha * (-b));
        pt1.y = cvRound(y0 + alpha * (a));
        pt2.x = cvRound(x0 - alpha * (-b));
        pt2.y = cvRound(y0 - alpha * (a));
        line( img, pt1, pt2, Scalar(0,0,255), 1.5, CV_AA);
    }
#endif

    Position::PUtils::ShowImage("origin",rot);
  

#if 0
    Rect rect;
    rect.x      = 0;
    rect.y      = img.rows >> 1;
    rect.width  = img.cols;
    rect.height = img.rows >> 1;

    Mat copy = img(rect).clone();

    cv::GaussianBlur(copy,copy,Size(5,5),0);

    Mat gray, binary;

    cvtColor(copy,gray,COLOR_BGR2GRAY);
    //二值化
    cv::threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);

    Mat mask = Mat::zeros(img.size(),CV_8UC1);
    binary.copyTo(mask(rect));
    Position::PUtils::ShowHImages("binary",std::vector<cv::Mat>{gray,binary});

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));

    Mat drawing = Mat::zeros(mask.size(), CV_8UC3);

    RNG rng(12345);

    for(size_t i = 0; i < contours.size(); ++i)
    {
        RotatedRect rrt = minAreaRect(contours[i]);
        int angle = abs(rrt.angle);
        if( angle < 20 || angle > 160 || angle == 90)
            continue;
        
        Scalar color = CV_RGB(255,0,0);// Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));

        drawContours(img,contours,i,color,2,8,hierarchy);

    }

    Position::PUtils::ShowImage("test",img);

#endif

    cv::waitKey(0);
TESTEND()