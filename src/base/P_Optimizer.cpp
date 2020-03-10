#include "P_Optimizer.h"
#include "P_Converter.h"
#include "P_MapPoint.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace Position
{

    //单例
    IOptimizer* IOptimizer::getSingleton()
    {
        static G2oOptimizer g2o;
        return &g2o;
    }
    //单张位姿优化
    int G2oOptimizer::frameOptimization(IKeyFrame *pFrame, const FloatVector &sigma2)
    {
        assert(pFrame && !pFrame->getPoints().empty());
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(PConverter::toSE3Quat(pFrame->getPose()));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        const int N = IFRAME(pFrame)->getKeySize();

        vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
        SzVector vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        const float deltaMono = sqrt(5.991);

        {
            // std::unique_lock<mutex> lock(MapPoint::mGlobalMutex);
            const MapPtVector &points = pFrame->getPoints();
            assert(points.size() <= N);
            for( int i = 0; i < N; ++i )
            {
                IMapPoint *pMppt = points[i];
                if( NULL != pMppt)
                {
                    //only monocular 
                    nInitialCorrespondences++;
                    // pFrame->mvbOutliner[i] = false;
                    Eigen::Matrix<double,2,1> obs;
                    const cv::KeyPoint &kp = IFRAME(pFrame)->getKeys()[i];
                    obs << kp.pt.x, kp.pt.y ;

                    g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);

                    const float invSigma2 = sigma2[kp.octave];

                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->fx = mFx;
                    e->fy = mFy;
                    e->cx = mCx;
                    e->cy = mCy;
                    cv::Mat Xw = pMppt->getWorldPos();
                    e->Xw[0] = Xw.at<MATTYPE>(0);
                    e->Xw[1] = Xw.at<MATTYPE>(1);
                    e->Xw[2] = Xw.at<MATTYPE>(2);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }

        if(nInitialCorrespondences<3)
            return 0;
        
        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4]={5.991,5.991,5.991,5.991};
        const int its[4]={10,10,10,10};    

        int nBad=0;
        for(size_t it=0; it<4; it++)
        {

            vSE3->setEstimate(PConverter::toSE3Quat(pFrame->getPose()));
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            nBad=0;
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

                // const size_t idx = vnIndexEdgeMono[i];

                // if(pFrame->mvbOutlier[idx])
                // {
                //     e->computeError();
                // }

                //卡方检验计算出的阈值(假设测量有一个像素的偏差)
                const float chi2 = e->chi2();

                if(chi2>chi2Mono[it])
                {                
                    // pFrame->mvbOutlier[idx]=true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    // pFrame->mvbOutlier[idx]=false;
                    e->setLevel(0);
                }

                if(it==2)
                    e->setRobustKernel(0);
            }

            if(optimizer.edges().size()<10)
                break;
        }    

        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        cv::Mat pose = PConverter::toCvMat(SE3quat_recov);
        pFrame->setPose(pose);
        cout << "bad pt size : " << nBad << endl;
        return nInitialCorrespondences-nBad;
    }

    //ba 优化
    void G2oOptimizer::bundleAdjustment(const KeyFrameVector &keyframes,const MapPtVector &mappts, const FloatVector &sigma2,int nIterations/* =5 */,
                                        bool *pbStopFlag /*= NULL*/,int nIndex /*= 0*/,bool bRobust /* = true */)
    {
        BolVector vbNotIncludedMp;
        vbNotIncludedMp.resize(mappts.size());

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        if(pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        u64  maxKFid = 0;

        for(size_t i = 0; i < keyframes.size();++i)
        {
            IKeyFrame *pKF = keyframes[i];
             if(pKF->isBad())
                 continue;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(PConverter::toSE3Quat(pKF->getPose()));
            const int fIndex = pKF->index();
            vSE3->setId(fIndex);
            vSE3->setFixed(fIndex == 0);
            optimizer.addVertex(vSE3);
            if(fIndex > maxKFid)
                maxKFid = fIndex;
        }

        //卡方测试参数
        const float thHuber2D = sqrt(5.99);

        for(size_t i = 0; i < mappts.size(); ++i)
        {
            IMapPoint *pMP = mappts[i];
            if(pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(PConverter::toVector3d(pMP->getWorldPos()));
            const int id = pMP->index() + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const KeyFrameMap observations = pMP->getObservations();

            int nEdges = 0;

            KeyFrameMap::const_iterator it = observations.begin();
            KeyFrameMap::const_iterator ed = observations.end();

            for(; it != ed; ++it )
            {
                IKeyFrame *pKF = it->first;
                if( pKF->isBad() || pKF->index() > maxKFid)
                    continue;

                nEdges++;

                const cv::KeyPoint &kp = IFRAME(pKF)->getKeys()[it->second];

                Eigen::Matrix<double,2,1> obs;
                obs << kp.pt.x, kp.pt.y;

                g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->index())));
                e->setMeasurement(obs);
                const float &invSigma2 = sigma2[kp.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = mFx;
                e->fy = mFy;
                e->cx = mCx;
                e->cy = mCy;

                optimizer.addEdge(e);

            }

            if(nEdges==0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMp[i] = true;
            }
            else
            {
                vbNotIncludedMp[i] = false;
            }
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);

        // Recover optimized data

        //Keyframes
        for(size_t i = 0; i < keyframes.size(); ++i)
        {
            IKeyFrame *pKF = keyframes[i];
            if(pKF->isBad())
                continue;

            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->index()));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            if(nIndex == 0)
            {
                pKF->setPose(PConverter::toCvMat(SE3quat));
            }
            else
            {
                // pKF->mTcwGBA.create(4,4,CV_32F);
                // PConverter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
                // pKF->mnBAGlobalForKF = nLoopKF;
            }
            
        }

        for(size_t i = 0; i < mappts.size(); ++i)
        {
            if(vbNotIncludedMp[i])
                continue;

            IMapPoint *pMP = mappts[i];

            if(pMP->isBad())
                continue;

            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->index() + maxKFid + 1));

            if(0 == nIndex)
            {
                pMP->setWorldPos(PConverter::toCvMat(vPoint->estimate()));
                // pMP->updateNormalAndDepth();
            }
            else
            {
                // pMP->mPosGBA.create(3,1,CV_32F);
                // PConverter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
                // pMP->mnBAGlobalForKF = nLoopKF;
            }
        }
    }
}