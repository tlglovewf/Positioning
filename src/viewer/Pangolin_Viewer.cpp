#include "Pangolin_Viewer.h"


namespace Position
{
    //绘制帧
    Mat CVFrameDrawer::drawFrame(IFrame *frame)
    {
        assert(frame);
        return Mat();
    }

     //绘制文本信息
    void CVFrameDrawer::drawTextInfo(cv::Mat &img, cv::Mat &txtimg)
    {

    }

     //构造函数
    Pangolin_Viwer::Pangolin_Viwer( std::shared_ptr<IConfig> pCfg , std::shared_ptr<IMap> pMap, std::shared_ptr<IPositioning> pPos):
    mCfg(pCfg),mMap(pMap),mPosition(pPos), mFDrawer(new CVFrameDrawer()), mbInit(false)
    {
        mWinW = GETCFGVALUE(mCfg,ViewerW,int);
        mWinH = GETCFGVALUE(mCfg,ViewerH,int);
        init();
    }

        //初始化
    void Pangolin_Viwer::init()
    {
        if(mbInit)
            return;
      
        pangolin::CreateWindowAndBind("Slam Simulator",mWinW,mWinH);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::View &menuPanel = pangolin::CreatePanel("menu").
                                    SetBounds(pangolin::Attach::Pix(0),pangolin::Attach::Pix(25),0.0,pangolin::Attach::Pix(mWinW));

        menuPanel.SetLayout(pangolin::LayoutEqualHorizontal);
        mbInit = true;
    }

        //绘制
    bool Pangolin_Viwer::render()
    {
        if(!mbInit)
            return false;

        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        pangolin::Var<bool> menuShowPoints("menu.MapPoints",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.MapFrames",true,true);
        pangolin::Var<bool> menuShowGraph("menu.CovGraph",true,true);
        pangolin::Var<bool> menuShowLines("menu.RelLines",true,true);

        // Define Camera Render Object (for view / scene browsing)
        // 
        float ViewptF =  GETCFGVALUE(mCfg,ViewptF,float);
        float ViewptX =  GETCFGVALUE(mCfg,ViewptX,float);
        float ViewptY =  GETCFGVALUE(mCfg,ViewptY,float);
        float ViewptZ =  GETCFGVALUE(mCfg,ViewptZ,float);

        pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(mWinW,mWinH,ViewptF,ViewptF,(mWinW >> 1),(mWinH >> 1),0.1,5000),
                    pangolin::ModelViewLookAt(ViewptX,ViewptY,ViewptZ, 0,0,0,0.0,-1.0, 0.0)
                    );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(pangolin::Attach::Pix(25), 1.0, 0, 1.0, -(float)mWinW/mWinH)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        bool bFollow = true;

        while(!pangolin::ShouldQuit())
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
           
            const float clr = 0.1f;
            glClearColor(clr, clr, clr ,1.0f);
            
            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow) 
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(ViewptX, ViewptY, ViewptZ, 0, 0, 0, 0.0, -1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else
            {
                bFollow = false;
            }
            d_cam.Activate(s_cam);

            if(menuShowKeyFrames)
            {
                drawFrames();
            }
            if(menuShowPoints)
            {
                drawMapPoints();
            }
            if(menuShowLines)
            {
                drawRelLines();
            }
            pangolin::FinishFrame();
            // cv::Mat img = mFDrawer->drawFrame(mPosition->currentFrame());
            // const int scale = 2;
            // cv::resize(img,img,cv::Size(img.cols >> scale,img.rows >> scale));
            // cv::imshow("CurFrame",img);
            // cv::waitKey(1);
        }

        return true;
    }

    void Nor(Mat &pt)
    {
        float d = cv::norm(pt);
        pt = pt / d;
    }

    static void suLookAt(GLdouble eyeX,GLdouble eyeY,GLdouble eyeZ,GLdouble centerX,GLdouble centerY,GLdouble centerZ,GLdouble upX,GLdouble upY,GLdouble upZ)
    {
    	GLdouble directMat[16];
    
    	for (int i = 0 ;i<16;i++)
    	{
    		directMat[i] = 0;
    	}
    
    	directMat[15]= 1;
    	Mat fvDirect = (Mat_<double>(3,1) << centerX-eyeX,centerY-eyeY,centerZ-eyeZ);
        Nor(fvDirect);
        Mat fvUpD = (Mat_<double>(3,1) << upX,upY,upZ);
        Nor(fvUpD);
        Mat fvC = fvDirect.cross(fvUpD);
        Nor(fvC);
        
        Mat fvUp = fvC.cross(fvDirect);
        Nor(fvUp);
    
    	fvDirect = -fvDirect;
    
    	directMat[0] = fvC.at<double>(0);
    	directMat[4] = fvC.at<double>(1);
    	directMat[8] = fvC.at<double>(2);
    	directMat[1] = fvUp.at<double>(0);
    	directMat[5] = fvUp.at<double>(1);
    	directMat[9] = fvUp.at<double>(2);
    	directMat[2] = fvDirect.at<double>(0);
    	directMat[6] = fvDirect.at<double>(1);
    	directMat[10] = fvDirect.at<double>(2);
    
    	glLoadMatrixd(directMat);
    
    	glTranslated(-eyeX,-eyeY,-eyeZ);
    }

    //绘制坐标轴
    static void drawCoordinateAxis(const cv::Point3f &pt)
    {
        glLineWidth(2.0);
       
        glBegin(GL_LINES);

        glColor3f(1.0f,0.0f,0.0f);
        glVertex3f(pt.x,pt.y,pt.z);
        glVertex3f(1,0,0);

        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(pt.x,pt.y,pt.z);
        glVertex3f(0,1,0);

        glColor3f(0.0f,0.0f,1.0f);
        glVertex3f(pt.x,pt.y,pt.z);
        glVertex3f(0,0,1);

        glEnd();
    }

    static void drawPose(const cv::Mat &pose)
    {
        Mat twl = pose.clone();

        cv::Mat Rcw = twl.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = twl.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        cv::Mat Ow = -Rwc*tcw;
        
        cv::Mat mWorldPosInv = cv::Mat::eye(4,4,CV_32F);
        Rwc.copyTo(mWorldPosInv.rowRange(0,3).colRange(0,3));
        Ow.copyTo(mWorldPosInv.rowRange(0,3).col(3));

        Mat t = mWorldPosInv.t();
        const float w = 0.5;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();
        glMultMatrixf(t.ptr<GLfloat>(0));
        glLineWidth(1);
        glColor3f(0.0f,1.0f,1.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);
        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();
        glPopMatrix();
    }

    //绘制帧
    void Pangolin_Viwer::drawFrames()
    {
        drawCoordinateAxis(Point3f(0,0,0));

        const float w = 0.5;
        const float h = w * 0.75;
        const float z = w * 0.6;
        const float mKeyFrameLineWidth = 1.0;

        const KeyFrameVector  vpKFs(mMap->getAllFrames());

        for(size_t i = 0; i < vpKFs.size(); ++i)
        {
            IFrame *pKF = vpKFs[i];
            cv::Mat Twc = pKF->getPose().clone();
            Twc = Twc.inv().t();
            // cout << i << " " << Twc << endl;
            glPushMatrix();

            glMultMatrixd(Twc.ptr<GLdouble>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    //绘制地图
    void Pangolin_Viwer::drawMapPoints()
    {
       const MapPtVector mpts = mMap->getAllMapPts();
       if(mpts.empty())
            return;

        glPointSize(2.0);
        glBegin(GL_POINTS);
        glColor3f(0.6,0.8,0.0);

        for(size_t i = 0;i < mpts.size();++i)
        {
            if(mpts[i]->isBad())
                continue;
            
            cv::Mat pos = mpts[i]->getPose();
            glVertex3f(pos.at<MATTYPE>(0),
                       pos.at<MATTYPE>(1),
                       pos.at<MATTYPE>(2));
        }
        glEnd();

    }

     //绘制关联线
    void Pangolin_Viwer::drawRelLines()
    {

    }
}