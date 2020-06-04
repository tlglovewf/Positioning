#include "P_PangolinViewer.h"
#include "P_IOHelper.h"

#ifdef USE_VIEW

namespace Position
{
    //构造函数
    Pangolin_Viewer::Pangolin_Viewer(const std::shared_ptr<IConfig> &pCfg) : mCfg(pCfg), mbInit(false), mbRender(false)
    {
        mWinW = 1080;   //GETCFGVALUE(mCfg, ViewerW, int);
        mWinH = 768;    //GETCFGVALUE(mCfg, ViewerH, int);
        mFrameViewer = std::unique_ptr<PFrameViewer>(new PFrameViewer());
    }
    
    //初始化
    void Pangolin_Viewer::init()
    {
        if (mbInit)
            return;
        pangolin::CreateWindowAndBind("Simulator", mWinW, mWinH);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::View &menuPanel = pangolin::CreatePanel("menu").SetBounds(pangolin::Attach::Pix(0), pangolin::Attach::Pix(25), 0.0, pangolin::Attach::Pix(mWinW));

        menuPanel.SetLayout(pangolin::LayoutEqualHorizontal);

        mViewF = 800;   //GETCFGVALUE(mCfg, ViewptF, float);
        mViewX = 0;     //GETCFGVALUE(mCfg, ViewptX, float);
        mViewY = -10;   //GETCFGVALUE(mCfg, ViewptY, float);
        mViewZ = -0.1;  //GETCFGVALUE(mCfg, ViewptZ, float);

        mCam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(mWinW, mWinH, mViewF, mViewF, (mWinW >> 1), (mWinH >> 1), 0.1, 5000),
            pangolin::ModelViewLookAt(mViewX, mViewY, mViewZ, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        mpView = &pangolin::CreateDisplay()
                      .SetBounds(pangolin::Attach::Pix(25), 1.0, 0, 1.0, -(float)mWinW / mWinH)
                      .SetHandler(new pangolin::Handler3D(mCam));
        PROMTD_S("init pangolin viewer successfully.");
        mbInit = true;
    }
    void Pangolin_Viewer::renderLoop()
    {
        init();
        while (renderOnce())
        {
            ;
        }
    
        PROMTD_S("Render Over !!!");
    }
    
    void GetCurrentOpenGLCameraMatrix(IKeyFrame *pkf, pangolin::OpenGlMatrix &M)
    {
        if (NULL == pkf)
            return;
        const Mat &pose = pkf->getPose().clone();
        if (pose.empty())
        {
            M.SetIdentity();
        }
        else
        {
            cv::Mat Rwc(3, 3, MATCVTYPE);
            cv::Mat twc(3, 1, MATCVTYPE);
    
            Rwc = pose.rowRange(0, 3).colRange(0, 3).t();
            twc = -Rwc * pose.rowRange(0, 3).col(3);
    
            M.m[0] = Rwc.at<MATTYPE>(0, 0);
            M.m[1] = Rwc.at<MATTYPE>(1, 0);
            M.m[2] = Rwc.at<MATTYPE>(2, 0);
            M.m[3] = 0.0;
    
            M.m[4] = Rwc.at<MATTYPE>(0, 1);
            M.m[5] = Rwc.at<MATTYPE>(1, 1);
            M.m[6] = Rwc.at<MATTYPE>(2, 1);
            M.m[7] = 0.0;
    
            M.m[8] = Rwc.at<MATTYPE>(0, 2);
            M.m[9] = Rwc.at<MATTYPE>(1, 2);
            M.m[10] = Rwc.at<MATTYPE>(2, 2);
            M.m[11] = 0.0;
    
            M.m[12] = twc.at<MATTYPE>(0);
            M.m[13] = twc.at<MATTYPE>(1);
            M.m[14] = twc.at<MATTYPE>(2);
            M.m[15] = 1.0;
        }
    }
    
    //绘制
    bool Pangolin_Viewer::renderOnce()
    {
        assert(mbInit);
        if (!mbInit || !mMap)
            return false;
        static pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        static pangolin::Var<bool> menuShowPoints("menu.MapPoints", true, true);
        static pangolin::Var<bool> menuShowKeyFrames("menu.MapFrames", true, true);
        static pangolin::Var<bool> menuShowTraceLine("menu.TraceLine", true, true);
        static pangolin::Var<bool> menuShowLines("menu.RelLines", true, true);
    
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();
        
        static bool bFollow = true;
    
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
            const float clr = 0.1f;
            glClearColor(clr, clr, clr, 1.0f);
            GetCurrentOpenGLCameraMatrix(mMap->currentKeyFrame(), Twc);
            if (menuFollowCamera && bFollow)
            {
                mCam.Follow(Twc);
            }
            else if (menuFollowCamera && !bFollow)
            {
                mCam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewX, mViewY, mViewZ, 0, 0, 0, 0.0, -1.0, 0.0));
                mCam.Follow(Twc);
                bFollow = true;
            }
            else
            {
                bFollow = false;
            }
            mpView->Activate(mCam);
            unique_lock<mutex> lock(mMap->mapUpdateMutex());
            if (menuShowKeyFrames)
            {
                drawFrames();
            }
            if (menuShowTraceLine)
            {
                drawTraceLine();
            }
            if (menuShowPoints)
            {
                drawMapPoints();
            }
            if (menuShowLines)
            {
                drawRelLines();
            }

            pangolin::FinishFrame();

            mFrameViewer->drawFrame();
        }
        mbRender = !pangolin::ShouldQuit();
        return mbRender;
    }
    
    void Nor(Mat &pt)
    {
        float d = cv::norm(pt);
        pt = pt / d;
    }
    
    static void suLookAt(GLdouble eyeX, GLdouble eyeY, GLdouble eyeZ, GLdouble centerX, GLdouble centerY, GLdouble centerZ, GLdouble upX, GLdouble upY, GLdouble upZ)
    {
        GLdouble directMat[16];

        for (int i = 0; i < 16; i++)
        {
            directMat[i] = 0;
        }

        directMat[15] = 1;
        Mat fvDirect = (Mat_<double>(3, 1) << centerX - eyeX, centerY - eyeY, centerZ - eyeZ);
        Nor(fvDirect);
        Mat fvUpD = (Mat_<double>(3, 1) << upX, upY, upZ);
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

        glTranslated(-eyeX, -eyeY, -eyeZ);
    }
    
    //绘制坐标轴
    static void drawCoordinateAxis(const cv::Point3f &pt)
    {
        glLineWidth(2.0);

        glBegin(GL_LINES);

        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(pt.x, pt.y, pt.z);
        glVertex3f(1, 0, 0);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(pt.x, pt.y, pt.z);
        glVertex3f(0, 1, 0);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(pt.x, pt.y, pt.z);
        glVertex3f(0, 0, 1);

        glEnd();
    }
    
    static void drawPose(const cv::Mat &pose)
    {
        Mat twl = pose.clone();

        cv::Mat Rcw = twl.rowRange(0, 3).colRange(0, 3);
        cv::Mat tcw = twl.rowRange(0, 3).col(3);
        cv::Mat Rwc = Rcw.t();
        cv::Mat Ow = -Rwc * tcw;

        cv::Mat mWorldPosInv = cv::Mat::eye(4, 4, MATCVTYPE);
        Rwc.copyTo(mWorldPosInv.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(mWorldPosInv.rowRange(0, 3).col(3));

        Mat t = mWorldPosInv.t();
        const float w = 0.5;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();
        glMultMatrixf(t.ptr<GLfloat>(0));
        glLineWidth(1);
        glColor3f(0.0f, 1.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();
    }
    
    //绘制帧
    void Pangolin_Viewer::drawFrames()
    {
        drawCoordinateAxis(Point3f(0, 0, 0));

        const float w = 0.5;
        const float h = w * 0.75;
        const float z = w * 0.6;
        const float framelineW = 1.0;
        KeyFrameVector vpKFs(mMap->getAllFrames());

        if (vpKFs.size() < 2)
            return;

        for (size_t i = 0; i < vpKFs.size(); ++i)
        {
            IKeyFrame *pKF = vpKFs[i];
            cv::Mat Twc = pKF->getPose().clone();
            if(Twc.empty())
                continue;
            Twc = Twc.inv().t();

            glPushMatrix();

            glMultMatrixd(Twc.ptr<GLdouble>(0));

            glLineWidth(framelineW);
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);

            glEnd();

            glPopMatrix();
        }
    }
    
    //绘制地图
    void Pangolin_Viewer::drawMapPoints()
    {
        const MapPtVector mpts = mMap->getAllMapPts();
        if (mpts.empty())
            return;

        glPointSize(2.0);
        glBegin(GL_POINTS);
        glColor3f(0.6, 0.8, 0.0);

        for (size_t i = 0; i < mpts.size(); ++i)
        {
            if (mpts[i]->isBad())
                continue;

            cv::Mat pos = mpts[i]->getWorldPos();
            glVertex3f(pos.at<MATTYPE>(0),
                       pos.at<MATTYPE>(1),
                       pos.at<MATTYPE>(2));
        }
        glEnd();
    }
    
    //绘制轨迹线
    void Pangolin_Viewer::drawTraceLine()
    {
        KeyFrameVector vpKFs(mMap->getAllFrames());
        
        const float framelineW = 1.0;
        glLineWidth(framelineW);
        glBegin(GL_LINE_STRIP);
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0, 0, 0);
        for (size_t i = 1; i < vpKFs.size(); ++i)
        {
            const Mat &p = vpKFs[i]->getCameraCenter();

            glVertex3f((float)p.at<MATTYPE>(0),
                       (float)p.at<MATTYPE>(1),
                       (float)p.at<MATTYPE>(2));
        }
        glEnd();
    }
    
    //绘制关联线
    void Pangolin_Viewer::drawRelLines()
    {
        IKeyFrame *pKf = mMap->currentKeyFrame();
        if (NULL == pKf)
            return;
        const cv::Mat &origin = pKf->getCameraCenter();
        const MapPtVector &mppoints = pKf->getWorldPoints();
        MapPtVector::const_iterator it = mppoints.begin();
        MapPtVector::const_iterator ed = mppoints.end();
        glColor3f(1.0, 1.0, 1.0);
        glLineWidth(0.5);
        glBegin(GL_LINES);
       
        for (; it != ed; ++it)
        {
            if ( (NULL == (*it)) || (*it)->isBad())
                continue;
            const cv::Mat &wdpos = (*it)->getWorldPos();
            glVertex3d(origin.at<MATTYPE>(0), origin.at<MATTYPE>(1), origin.at<MATTYPE>(2));
            glVertex3d(wdpos.at<MATTYPE>(0), wdpos.at<MATTYPE>(1), wdpos.at<MATTYPE>(2));
        }
        glEnd();
    }
} // namespace Position

#endif