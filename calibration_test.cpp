#include <iostream>
#include "eigen3/Eigen/Dense"
#include <math.h>
#include <float.h>
using namespace std;
using namespace Eigen;

Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R, int isread = 1);

Vector3d R_to_eulerAngle(Matrix3d R){

    //roll-->yaw-->pitch顺规构造的旋转矩阵 
    /*Matrix3d R_roll_yaw_pitch;
    R_roll_yaw_pitch << cos(pitch) * cos(yaw), -cos(pitch) * sin(yaw) * cos(roll) + sin(pitch) * sin(roll), cos(pitch) * sin(yaw) * sin(roll)+sin(pitch) * cos(roll),
                sin(yaw), cos(yaw) * cos(roll), -cos(yaw) * sin(roll),
                -sin(pitch) * cos(yaw), sin(pitch)*sin(yaw) * cos(roll)+cos(pitch) * sin(roll), -sin(pitch) * sin(yaw) * sin(roll)+cos(pitch) * cos(roll);*/
    
    /*R_pitch_yaw_roll << cos(yaw)*cos(pitch), -sin(yaw), cos(yaw)*sin(pitch),
                cos(roll)*sin(yaw)*cos(pitch)+sin(roll)*sin(pitch), cos(roll)*cos(yaw), cos(roll)*sin(yaw)*sin(pitch)-sin(roll)*cos(pitch),
                sin(roll)*sin(yaw)*cos(pitch)-cos(roll)*sin(pitch), sin(roll)*cos(yaw), sin(roll)*sin(yaw)*sin(pitch)+cos(roll)*cos(pitch);*/

    /*
    R_yaw_pitch_roll << cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch),
			sin(roll)*sin(pitch)*cos(yaw)+cos(roll)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), -sin(roll)*cos(pitch),
			-cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw), cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw), cos(roll)*cos(pitch)
    */
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

/*Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R)
{
    
    //旋转顺序是roll --> pitch --> yaw
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

    Matrix3d R_result;
    R_result << cos(p)*cos(y), sin(r)*sin(p)*cos(y)-cos(r)*sin(y), cos(r)*sin(p)*cos(y)+sin(r)*sin(y),
		cos(p)*sin(y), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), cos(r)*sin(p)*sin(y)-sin(r)*cos(y),
		-sin(p), sin(r)*cos(p), cos(r)*cos(p);
    std::cout << "R_rpy:" << endl;
    std::cout << R_result << endl;  

    return ypr;
}*/

//最全面的解法
Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R, int israd)
{
	const float pi = 3.14159265397932384626433;
	float theta = 0, psi = 0, pfi = 0;
	float pitch = 0, roll = 0, yaw = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN){ // abs(R(2, 0)) != 1
		float pitch_1 = -asin(R(2, 0));
		float pitch_2 = pi - pitch_1;
		float roll_1 = atan2(R(2,1)/cos(pitch_1), R(2,2)/cos(pitch_1));
		float roll_2 = atan2(R(2,0)/cos(pitch_2), R(2,2)/cos(pitch_2));
		float yaw_1 = atan2(R(1,0)/cos(pitch_1), R(0,0)/cos(pitch_1));
		float yaw_2 = atan2(R(1,0)/cos(pitch_2), R(0,0)/cos(pitch_2));
		std::cout << "pitch_1:" << pitch_1 << " roll_1:" << roll_1 << " yaw_1:" << yaw_1 << endl;
		std::cout << "pitch_2:" << pitch_2 << " roll_2:" << roll_2 << " yaw_2:" << yaw_2 << endl;
		pitch = pitch_1;
		roll = roll_1;
		yaw = yaw_1;
	} else{
		yaw = 0;
		float delta = atan2(R(0,1), R(0,2));
		//if (R(2, 0) > -1 - FLT_MIN && R(2, 0) < -1 + FLT_MIN){ // R(2,0) == -1
		if (R(2, 0) < 0 ){ // R(2,0) == -1
			std::cout << "pitch is pi/2" << endl;
			pitch = pi / 2;
			roll = yaw + delta;
		} else{
			std::cout << "pitch is -pi/2" << endl;
			pitch = -pi / 2;
			roll = -yaw + delta;
		}
	}
    
	Eigen::Vector3d ypr(3);
 
	// psi is along x-axis, theta is along y-axis, pfi is along z axis
	if (israd){ // for rad 
		ypr[0] = yaw;
		ypr[1] = pitch;
		ypr[2] = roll;
	} else{
		ypr[0] = yaw * 180 / pi;
		ypr[1] = pitch * 180 / pi;
		ypr[2] = roll * 180 / pi;
	}
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

int main()
{


    double roll = 0.028468914;
    //double pitch = 0.023796266;
    double pitch = M_PI / 2;
    double yaw = -0.072162271;

    std::cout << "roll:" << roll << " pitch:"
    << pitch << " yaw:" << yaw << std::endl;

    //旋转顺序是yaw-->pitch-->roll
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

    cout << "R_yaw:" << endl;
    cout << R_yaw << endl;
    cout << "R_pitch:" << endl;
    cout << R_pitch << endl;
    cout << "R_roll:" << endl;
    cout << R_roll << endl;

    // roll->pitch->yaw 顺序 
    std::cout << "已知顺规roll-pitch-yaw， 构造旋转矩阵，先旋转的在右边" << std::endl;
    Matrix3d R = R_yaw * R_pitch * R_roll;
    std::cout << "旋转矩阵:" << endl;
    std::cout << R << endl;
    std::cout << "已知顺规roll-pitch-yaw， 反向求解欧拉角" << std::endl;
    Vector3d ypr = R2rpy(R);
    std::cout << "yaw:"<< ypr[0] << " pitch:" << ypr[1] << " roll:" << ypr[2] << std::endl;

    std::cout << "Test roll-pitch-yaw" << std::endl;
     R << 0,-1,0,
          1, 0, 0,
          0,0,1;
}
