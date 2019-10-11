#include <iostream>
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "utility.h"
using namespace std;
using namespace Eigen;

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R)
{
    //旋转顺序是yaw-->pitch-->roll，通过旋转矩阵求的欧拉角
    //R_roll*R_pitch*R_yaw
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

Matrix3d Quaterniond_to_R(Quaterniond q){
    cout << "Quaterniond ==> R" <<endl;
    cout << "Quaterniond: w:" << q.w() << " x:" << q.x() 
        << " y:" << q.y() << " z:" << q.z() << std::endl;
    //原始计算
    //从四元素到旋转矩阵
    //R(q) = [
    //        1-2y^2-2z^2  2xy-2zw,      2xz+2yw
    //        2xy+2zw      1-2x^2-2z^2   2yz-2xw
    //        2xz-2yq      2yz+2xw       1-2x^2-2y^2
    //       ]
    double x = q.x();
    double y = q.y();
    double z = q.z();
    double w = q.w();
    Matrix3d R;
    R << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 *y*w, 2*x*y+2*z*w,1-2*x*x-2*z*z, 2*y*z-2*x*w, 2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y;
    cout << "quaterniond to R:" << R << endl; 
    //Eigen方式
    Eigen::Matrix3d R_trix = q.toRotationMatrix();
    cout << "Eigen quaterniond to R:" << R_trix;
    return R;
}

Quaterniond R_to_Quaterniond(Matrix3d R){
    cout << "R ==> Quaterniond" <<endl;
    Quaterniond q;
    //由旋转矩阵转换到四元数
    double w = sqrt(1+ R(0, 0) + R(1, 1) + R(2, 2)) / 2;
    double x= (R(2,1) - R(1,2))/ 4 * w;
    double y= (R(0,2) - R(2,0))/ 4 * w;
    double z= (R(1,0) - R(0,1))/ 4 * w;
    //有些误差，什么原因？
    std::cout << "R to quaterniond: x-->" << x << " y:" << y << 
    " z:" << z << " w:" << w << std::endl;
    q = Quaterniond(w, x, y, z);

    //Eigen 没有误差
    Quaterniond q1 = Quaterniond(R);
    std::cout << "Eigen R to quaterniond: x-->" << q1.x() << " y:" << q1.y() << 
    " z:" << q1.z() << " w:" << q1.w() << std::endl;
    return q;
}

Matrix3d eulerAngle_to_R(double yaw, double pitch, double roll){
    Matrix3d R;
    cout << "eulerAngles ==> Rotation" << endl;
    std::cout << "yaw:" << yaw << " pitch:"
    << pitch << " roll:" << roll << std::endl;
    //顺规：欧拉角的旋转顺序
    //由欧拉角到旋转矩阵与顺规有关，以下是pitch-->yaw-->roll顺序构造旋转矩阵
    //谁先旋转谁的矩阵在右边
    Matrix3d R_pitch = pitch_to_R(pitch);
    Matrix3d R_yaw = yaw_to_R(yaw);
    Matrix3d R_roll = roll_to_R(roll);
    R = R_roll * R_yaw * R_pitch;
    std::cout << "R:" << R<< std::endl;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    //四元数转换顺序矩阵相乘顺序不变
    Quaterniond q = rollAngle * yawAngle * pitchAngle;
    Matrix3d R1 = q.toRotationMatrix();
    std::cout << "Eigen R:" << R1 << std::endl;
    return R;
}

Vector3d R_to_eulerAngle(Matrix3d R){

    //roll-->yaw-->pitch顺规构造的旋转矩阵 
    /*Matrix3d R_roll_yaw_pitch;
    R_roll_yaw_pitch << cos(pitch) * cos(yaw), -cos(pitch) * sin(yaw) * cos(roll) + sin(pitch) * sin(roll), cos(pitch) * sin(yaw) * sin(roll)+sin(pitch) * cos(roll),
                sin(yaw), cos(yaw) * cos(roll), -cos(yaw) * sin(roll),
                -sin(pitch) * cos(yaw), sin(pitch)*sin(yaw) * cos(roll)+cos(pitch) * sin(roll), -sin(pitch) * sin(yaw) * sin(roll)+cos(pitch) * cos(roll);*/
    
    /*R_pitch_yaw_roll << cos(yaw)*cos(pitch), -sin(yaw), cos(yaw)*sin(pitch),
                cos(roll)*sin(yaw)*cos(pitch)+sin(roll)*sin(pitch), cos(roll)*cos(yaw), cos(roll)*sin(yaw)*sin(pitch)-sin(roll)*cos(pitch),
                sin(roll)*sin(yaw)*cos(pitch)-cos(roll)*sin(pitch), sin(roll)*cos(yaw), sin(roll)*sin(yaw)*sin(pitch)+cos(roll)*cos(pitch);*/
    Vector3d ypr;
    double yaw, pitch, roll;
    cout << "Rotation ==> eulaerAngles" << endl;
    //三角函数重要公式 x=atan2(sin(x), cos(x));
   //sin(-p)=-sin(p) cos(-p)=cos(p)
    yaw = asin(-R(0, 1));
    pitch = atan2(R(0, 2), R(0, 0));
    roll = atan2(R(2, 1), R(1, 1));

    //
    /*cout << "cos(yaw):" << cos(yaw) << endl;
    double pitch_1 = acos(R(0, 0) /  cos(yaw));
    cout <<  "pitch_1:" << pitch_1 << endl;
    //R(0, 0) = cos(pitch)*cos(yaw) > 0
    //R(0, 2) = cos(yaw)*sin(pitch) < 0
    // 由此得出pitch<0, 则pitch应该取反
    cout << "adjust pitch:" << -pitch_1 << endl;
    //roll同理，因此使用atan2考虑两个值，则无此问题 
    double roll_1 = acos(R(1, 1) / cos(yaw));
    cout << "adjust roll:" << -roll_1 << endl;*/

    ypr(0) = yaw;
    ypr(1) = pitch;
    ypr(2) = roll;
    std::cout << "yaw:" << yaw << " pitch:" << pitch << " roll:" << roll << std::endl;
    return ypr;
}

AngleAxisd R_to_AngleAxisd(Matrix3d R){
    cout << "R==>AngleAxisd" << endl;
    //使用旋转矩阵转旋转向量的方式
    //方法1 
    //转角计算
    double theta = acos((R(0, 0) + R(1,1) + R(2,2) - 1) / 2);
    cout << "Angle:" << theta << endl;
    //转轴计算 
    //1/2*sin(theta)  [ r32 - r23
    //                  r13 - r31
    //                  r21 - r12
    //                ]
    double x1 = 1 / (2 * sin(theta));
    Vector3d r = Vector3d(R(2,1) - R(1, 2), R(0,2) - R(2,0), R(1,0)-R(0,1)) * x1;
    cout << "Rotation Vector:" << r.transpose() << endl;
    
    //Eigen 从旋转矩阵到旋转向量
    AngleAxisd V;
    V.fromRotationMatrix(R);
    cout << "Eigen Angle:" << V.angle() << endl;
    cout << "Eigen Rotation Vector:" << V.axis().transpose() << endl;
    return V;
}

Matrix3d  AngleAxisd_to_R(AngleAxisd P){
    //使用如下公式可以从轴角转换成旋转矩阵
    cout << "AngleAxisd ==> R" << endl;
    Vector3d r = P.axis();
    double theta = P.angle();
    double rx = r(0);
    double ry=r(1);
    double rz=r(2);
    double cos_x = cos(theta);
    double sin_x = sin(theta);
    Matrix3d R;
    R << rx*rx*(1-cos_x)+cos_x, rx*ry*(1-cos_x)-rz*sin_x, rx*rz*(1-cos_x)+ry*sin_x,
            rx*ry*(1-cos_x)+rz*sin_x, ry*ry*(1-cos_x)+cos_x, ry*rz*(1-cos_x)-rx*sin_x,
            rx*rz*(1-cos_x)-ry*sin_x, ry*rz*(1-cos_x)+rx*sin_x, rz*rz*(1-cos_x)+cos_x;
   cout << "Rotation:"<<R<< endl;

    //使用Eigen可以从轴角直接转换成旋转矩阵 
    cout << "Eigen Rotation:" << endl << P.matrix() << endl;

    Eigen::Quaterniond q = Quaterniond(P.matrix());
    //使用四元数来对旋转向量进行赋值
    //方法1
    AngleAxisd V;
    V = q;
    cout << "Eigen Rotation by Quaterniond:" << endl << V.matrix() << endl;
 
    return R;
}

void test_eigen_eulerAngle(double yaw, double pitch ,double roll){
    //无法从四元素恢复回欧拉角?
    cout << "just test eigen aulerAngles" << endl;
    Eigen::Vector3f  test_angle(roll, pitch, yaw);
    cout << "roll:" << test_angle(0) << " pitch:" << test_angle(1) << 
        " yaw:" << test_angle(2)<< endl;

    Eigen::Quaterniond q_test = Eigen::AngleAxisd(test_angle[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(test_angle[1], Eigen::Vector3d::UnitY()) * 
    Eigen::AngleAxisd(test_angle[2], Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d RXYZ = q_test.toRotationMatrix();
    Eigen::Vector3d EulerAngleZYX = RXYZ.eulerAngles(2, 1, 0);

    cout << "roll:" << EulerAngleZYX(0) << " pitch:" << EulerAngleZYX(1) << 
        " yaw:" << EulerAngleZYX(2)<< endl;

    double roll1 = test_angle.x();
    double pitch1 = test_angle.y();
    double yaw1 = test_angle.z();
    //得到的欧拉角不等于原始的欧拉角，这里生成的四元素也不等于原来的四元素
    Eigen::AngleAxisd rollAngle1(roll1, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle1(pitch1, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle1(yaw1, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q1 = pitchAngle1 * yawAngle1  * rollAngle1;
    cout << "quaterniod q1-->" << "x:" << q1.x() << " y:" << q1.y() << 
        " z:" << q1.z() << " w:" << q1.w() << endl;
}

int main()
{
    double roll = -0.0092940517;
    double pitch = -0.23466094;
    double yaw = -0.07634446;
    Matrix3d R = eulerAngle_to_R(yaw, pitch, roll);
    std::cout << std::endl;
    std::cout << std::endl;

    R_to_eulerAngle(R);
    std::cout << std::endl;
    std::cout << std::endl;
    
    Quaterniond q = Quaterniond(R);
    Quaterniond_to_R(q);
    cout << endl;
    cout << endl;

    R_to_Quaterniond(R);
    cout << endl;
    cout << endl;

    AngleAxisd p = R_to_AngleAxisd(R);
    cout << endl;
    cout << endl;
    AngleAxisd_to_R(p);
    cout << endl;
    cout << endl;
}
