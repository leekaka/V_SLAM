#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>


using namespace std;

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST (double x, double y): _x ( x ), _y ( y ) {}
    template <typename T>
    bool operator() 
    ( const T* const abc,T* residual )
    const
	{
	    residual[0] = T ( _y ) - ceres::exp ( abc[0]*T( _x ) *T( _x ) + abc[1]*T ( _x ) + abc[2]);
	    return true;
	}
    const double _x, _y;
};

int main( int argc,char**argv)
{
    double a=1.0,b=2.0,c=1.0;
    int N=100;
    double w_sigma=1.0;
    cv::RNG rng;   //opencv 随机数产生器
    double abc[3] = {0, 0, 0};
    vector<double>x_data, y_data;

    cout<<"generating data"<<endl;
    for ( int i=0;i<N;i++)
    {
	double x = i/100.0;
	x_data.push_back( x );
	y_data.push_back( 
		exp(a*x*x + b*x + c)+rng.gaussian( w_sigma)
		);
	cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    // 构建最小二乘问题
    ceres::Problem problem;
    for (int i=0;i<N;i++)
    {  // 自动求导，模板参数：误差类型，输入维度，输出维度，唯数要和struct一致
	problem.AddResidualBlock(
		new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
		   new CURVE_FITTING_COST( x_data[i], y_data[i])
		    ),
		nullptr,
		abc
		);
    }

    // 配置
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    
    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    
    ceres::Solve (options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);

    // 输出结果
    cout<<"eastimate a, b, c = ";
    for (auto a:abc )cout<<a<<" ";
    cout<<endl;

    return 0;

}
