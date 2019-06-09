#include "../include/common.h"



    

bool poseEstimationDirect(const vector<Measurement>& measurements, cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw )
{
    //初始化过g2o  误差项优化变量为6维，误差值维度为1维
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  //求解向量是6*1的
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock* solver_ptr = new DirectBlock (unique_ptr<DirectBlock::LinearSolverType>(linearSolver));

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (unique_ptr<DirectBlock>(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm (solver);
    optimizer.setVerbose(true);


    //增加定点，优化参数
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate (g2o:: SE3Quat(Tcw.rotation(),Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    int id =1;
    for (Measurement m :measurements )
    {
	EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(
		m.pos_world,
		K(0,0), K(1,1), K(0,2), K(1,2), gray
		);
	edge->setVertex(0,pose);
	edge->setMeasurement(m.grayscale);
	edge->setInformation (Eigen::Matrix<double,1,1>::Identity() );
	edge->setId(id++);
	optimizer.addEdge(edge);
    }
    cout<<"edges in graph: "<<optimizer.edges().size()<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    Tcw = pose->estimate();
}
















