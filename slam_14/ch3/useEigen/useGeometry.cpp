#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>



int main( int argc, char** argv)
{
    // 旋转向量
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector (M_PI/3, Eigen::Vector3d (0, 0, 1));
    cout .precision(3);


    cout << "rotation matrix = \n "<<rotation_vector.matrix() << endl << endl;



    rotation_matrix = rotation_vector.toRotationMatrix();
    cout << "toRotationMatrix = \n "<<rotation_matrix <<endl;
    // 用 AngleAxied 可以进行坐标变换    

    Eigen::Vector3d v (1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after ratotion = " << v_rotated.transpose() << endl;

    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // 欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles( 2,1,0); // ZYX yaw , pitch, roll
    cout << "yaw pitch roll = " <<euler_angles.transpose()<<endl;

    // 欧式变换矩阵使用Eigen::isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();

    T.rotate ( rotation_vector); // 按照rotation_vector旋转
    T.pretranslate( Eigen::Vector3d( 0,0,0)); //把平移向量设为1,3,4
    cout<<"Transform matrix = \n" << T.matrix() << endl;

    //用变换矩阵进行变换
    Eigen::Vector3d v_transformed = T * v;
    cout<<"v_transformed = " << v_transformed << endl;

    //四元素
    Eigen::Quaterniond q = Eigen::Quaterniond( rotation_vector);
    cout<<"quaternion vector = \n" <<q.coeffs() <<endl;

    q = Eigen::Quaterniond (rotation_matrix);
    cout<<"Quaterniond matrix = \n"<<q.coeffs() <<endl;

    v_rotated = q * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;


    return 0;
}



