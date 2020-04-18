#include <iostream>
#include "eigen3/Eigen/Dense"
#include <math.h>
using namespace std;
using namespace Eigen;

Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R)
{
    //旋转顺序时roll --> pitch --> yaw
    //R_yaw*R_pitch*R_roll
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    std::cout << "y:" << y << " p:" << p << " r:" << r << std::endl;

    return ypr;
}

Matrix3d yaw_to_R(double yaw){
    Matrix3d R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
    return R_yaw;
}

Matrix3d pitch_to_R(double pitch){
    Matrix3d R_pitch;
    R_pitch << cos(pitch), 0, sin(pitch),
                0, 1, 0,
                -sin(pitch), 0, cos(pitch);
    return R_pitch;
}

Matrix3d roll_to_R(double roll){
    Matrix3d R_roll;
    R_roll << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);
    return R_roll;
}

int main()
{


    //double roll = -0.0092940517;
    //double pitch = -0.23466094;
    //double yaw = -0.07634446;
    double roll = 0.79070538;
    double pitch = 1.3794478;
    double yaw = 1.6139573;
    //double roll = 1.6139573;
    //double pitch = 1.3794478;
    //double yaw = 0.79070538;

    std::cout << "initail yaw:" << yaw << " pitch:"
    << pitch << " roll:" << roll << std::endl;

    //旋转顺序是pitch-->yaw-->roll
    //验证矩阵方式转换,通过以下方式得到的四元数和使用Eigen的AngleAxisd得到四元数相同
    Matrix3d R_roll;
    Matrix3d R_yaw;
    Matrix3d R_pitch;
    R_roll << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);
    R_yaw << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
    R_pitch << cos(pitch), 0, sin(pitch),
                0, 1, 0,
                -sin(pitch), 0, cos(pitch);

     // roll->pitch->yaw顺序
     Matrix3d R = R_yaw * R_pitch * R_roll;
     R2rpy(R);

     std::cout << "Test roll-pitch-yaw" << std::endl;
     R << 0,-1,0,
          1, 0, 0,
          0,0,1;
    R2rpy(R);
    //构造旋转矩阵的方式 (顺规)
    //欧拉角构造旋转矩阵的顺序 谁先旋转谁的矩阵在右边，因此以下是pitch-->yaw-->roll的顺序
    
    Matrix3d R2 =  R_roll * R_yaw * R_pitch;
    Matrix3d R_result2;
    R_result2 << cos(yaw)*cos(pitch), -sin(yaw), cos(yaw)*sin(pitch),
                cos(roll)*sin(yaw)*cos(pitch)+sin(roll)*sin(pitch), cos(roll)*cos(yaw), cos(roll)*sin(yaw)*sin(pitch)-sin(roll)*cos(pitch),
                sin(roll)*sin(yaw)*cos(pitch)-cos(roll)*sin(pitch), sin(roll)*cos(yaw), sin(roll)*sin(yaw)*sin(pitch)+cos(roll)*cos(pitch);
    cout << "R_result2:" << R_result2 << endl;
    std::cout << "the same R2_trix:" << R2 << std::endl;

    Matrix3d R_cam_to_vehicle;
    R_cam_to_vehicle << 0, 0, 1,
                     -1, 0,0,
                     0,-1,0;
    
    cout << "camera to vehicle rotation:" << R_cam_to_vehicle << endl;
    R_result2 = R_cam_to_vehicle;
    
    
    cout << "R_result2:" << R_result2 << endl; 
    // pitch --> yaw --> roll顺序下欧拉角的求解 
    cout << "yaw:" << asin(-R_result2(0, 1)) << std::endl;
    std::cout <<"pitch:" << atan2(R_result2(0, 2), R_result2(0, 0)) << std::endl;
    std::cout << "roll:" << atan2(R_result2(2, 1), R_result2(1, 1)) << std::endl;

    //转点方式时yaw pitch roll对应z y x轴不变
    //此时的yaw pitch roll的物理意义不存在
    //不太理解
    Vector3d pos;
    pos << 1, 1,1; 
    cout << "initail pos:" << pos.transpose() << endl;
    Vector3d pos_1 = pitch_to_R(90*M_PI/180) * pos; 
    cout << "1. pitch 90-->" << pos_1.transpose() << endl; 
    Vector3d pos_2 = yaw_to_R(0) * pos_1; 
    cout << "2. yaw 0-->" << pos_2.transpose() << endl; 
    Vector3d pos_3 = roll_to_R(-90*M_PI/180) * pos_2; 
    cout << "3. roll -90-->" << pos_3.transpose() << endl; 
   
    double yaw_1 = asin(-R_result2(0, 1));
    double pitch_1 = atan2(R_result2(0, 2), R_result2(0, 0));
    double roll_1 = atan2(R_result2(2, 1), R_result2(1, 1));
    Matrix3d R1 = roll_to_R(roll_1) * yaw_to_R(yaw_1) * pitch_to_R(pitch_1);
    cout << "R1:" << R1 << endl;
    // 从旋转矩阵到四元数    
    Eigen::Quaterniond q2 = Quaterniond(R_result2);
    cout << "quaterniond q2: x:" << q2.x() << " y:" << q2.y() << " z:" << q2.z() << " w:" << q2.w() << endl;
    
    //从右手坐标系的四元数转到左手坐标系的四元数
    //roll -- pitch --yaw 顺规从四元素转到欧拉角
    Quaterniond q_unity = Quaterniond(q2.w(), q2.y(), -q2.z(), -q2.x());
    //Quaterniond q_unity = Quaterniond(q2.w(), -q2.y(), q2.z(), q2.x());
    //Quaterniond q_unity = q2;
    Matrix3d R_unity_cam_to_vehicle;
    R_unity_cam_to_vehicle << 1, 0, 0,
                     0, -1,0,
                     0,0,1;
    Matrix3d R_revert;
    Matrix3d R_unity = q_unity.toRotationMatrix();
    Vector3d ypr = R2rpy(R_unity);
    
    cout << "quaterniond q_unity: x:" << q_unity.x() << " y:" << q_unity.y() << " z:" << q_unity.z() << " w:" << q_unity.w() << endl;

    std::cout << "R_unit:" << R_unity << endl;
    double pitch_unity = asin(-R_unity(2,0));
    cout << "pitch_unity:" << pitch_unity << " pitch_degree:" << pitch_unity * 180 / M_PI << endl;
    double yaw_unity = atan2(R_unity(1,0), R_unity(0,0));
    cout << "yaw_unity:" << yaw_unity << " yaw_degree:" << yaw_unity * 180 / M_PI << endl;
    double roll_unity = atan2(R_unity(2,1), R_unity(2,2));
    cout << "roll_unity:" << roll_unity << " roll_degree:" << roll_unity * 180 / M_PI<< endl;

    cout << endl;
    cout << endl;
}
