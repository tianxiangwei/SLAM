#include <iostream>
#include "eigen3/Eigen/Dense"
#include <math.h>
#include <float.h>
using namespace std;
using namespace Eigen;

//Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R, int isread = 1);
Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R);

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R);

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R){

    //yaw-->pitch-->roll顺规构造的旋转矩阵 
    Vector3d ypr;
    double yaw, pitch, roll;
    //三角函数重要公式 x=atan2(sin(x), cos(x));
   //sin(-p)=-sin(p) cos(-p)=cos(p)
   //Sin(A+B)=SinA*CosB+SinB*CosA
   //Sin(A-B)=SinA*CosB-SinB*CosA
   //Cos(A+B)=CosA*CosB-SinA*SinB
   //Cos(A-B)=CosA*CosB+SinA*SinB
   if (abs(R(0, 2)) < 1 - FLT_MIN || abs(R(0, 2)) > 1 + FLT_MIN){ // abs(R(2, 0)) != 1
	float pitch_1 = asin(R(0, 2));
	float pitch_2 = M_PI - pitch_1;
	float roll_1 = atan2(-R(1,2)/cos(pitch_1), R(2,2)/cos(pitch_1));
	float roll_2 = atan2(-R(1,0)/cos(pitch_2), R(2,2)/cos(pitch_2));
	float yaw_1 = atan2(-R(0,1)/cos(pitch_1), R(0,0)/cos(pitch_1));
	float yaw_2 = atan2(-R(0,1)/cos(pitch_2), R(0,0)/cos(pitch_2));
	std::cout << "yaw_1:" << yaw_1 << " pitch_1:" << pitch_1 << " roll_1:" << roll_1 << endl;
	std::cout << "yaw_2:" << yaw_2 << " pitch_2:" << pitch_2 << " roll_2:" << roll_2 << endl;
	pitch = pitch_1;
	roll = roll_1;
	yaw = yaw_1;
    }
    else{
	if (sin(pitch) > 0 ){
	   std::cout << "pitch is pi/2" << endl;
	   pitch = M_PI / 2;
           //R(1,0)=R(2,1)=sin(roll+yaw)
           //R(1,1) = cos(roll-yaw)
	   //R(2,0) = sin(roll)*sin(yaw) - cos(roll)*cos(yaw)
	   //-R(2,0) = cos(roll+yaw)
	   double delta = atan2(R(1,0),-R(2,0)); // roll+yaw
           double delphi = acos(R(1,1)); // roll-yaw
           roll = (delta + delphi) / 2;
	   yaw = delta - roll;
	   std::cout << "roll:" << roll << endl;
	   std::cout << "yaw:" << yaw << endl;
    	   
	   //万向琐问题，只要满足以下条件存在无穷多的解
           std::cout << "delta(roll+yaw):" << delta << endl; 
           std::cout << "delphi(roll-yaw):" << delphi << endl; 
          
           std::cout << "sin(roll+yaw) = R(1,0) = R(2,1):" << sin(roll+yaw) << endl;
           std::cout << "cos(roll+yaw) = R(1,1):" << cos(roll+yaw) << endl;
           std::cout << "cos(roll-yaw) = -R(2,0):" << cos(roll-yaw) << endl;
	}
        else{
	   std::cout << "pitch is -pi/2" << endl;
	   pitch = - M_PI / 2;
           //R(1,0) = -R(2,1) = sin(yaw-roll)
	   //R(1,1) = cos(roll+yaw)
	   //R(2,0) = cos(roll-yaw) = cos(yaw-roll)
           double delta = atan2(R(1,0), R(2,0));
	   double delphi = acos(R(1,1));
           yaw = (delta + delphi) /2;
           roll = delta - yaw;
	}
    }

    ypr(0) = yaw;
    ypr(1) = pitch;
    ypr(2) = roll;
    
    Matrix3d R_yaw_pitch_roll;
    R_yaw_pitch_roll << cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch),
			sin(roll)*sin(pitch)*cos(yaw)+cos(roll)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), -sin(roll)*cos(pitch),
			-cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw), cos(roll)*sin(pitch)*sin(yaw)+sin(roll)*cos(yaw), cos(roll)*cos(pitch);
    std::cout << "R_ypr:" << endl;
    std::cout << R_yaw_pitch_roll << endl;  
    return ypr;
}


//该解法在ptich=+-90度时，仍能得到正确的结果。
Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R)
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
    
   //std::cout << "yaw:" << y << " pitch:" << p << " roll:" << r << endl;
   //std::cout << "cos(p):" << cos(p) << endl;

    Matrix3d R_result;
    R_result << cos(p)*cos(y), sin(r)*sin(p)*cos(y)-cos(r)*sin(y), cos(r)*sin(p)*cos(y)+sin(r)*sin(y),
		cos(p)*sin(y), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), cos(r)*sin(p)*sin(y)-sin(r)*cos(y),
		-sin(p), sin(r)*cos(p), cos(r)*cos(p);
    std::cout << "R_rpy:" << endl;
    std::cout << R_result << endl;  

    return ypr;
}

//最全面的解法
/*Eigen::Vector3d R2rpy(const Eigen::Matrix3d& R, int israd)
{
	const float pi = 3.14159265397932384626433;
	float pitch = 0, roll = 0, yaw = 0;
	if (abs(R(2, 0)) < 1 - FLT_MIN || abs(R(2, 0)) > 1 + FLT_MIN){ // abs(R(2, 0)) != 1
		float pitch_1 = -asin(R(2, 0));
		float pitch_2 = pi - pitch_1;
		float roll_1 = atan2(R(2,1)/cos(pitch_1), R(2,2)/cos(pitch_1));
		float roll_2 = atan2(R(2,0)/cos(pitch_2), R(2,2)/cos(pitch_2));
		float yaw_1 = atan2(R(1,0)/cos(pitch_1), R(0,0)/cos(pitch_1));
		float yaw_2 = atan2(R(1,0)/cos(pitch_2), R(0,0)/cos(pitch_2));
		std::cout << "yaw_1:" << yaw_1 << " pitch_1:" << pitch_1 << " roll_1:" << roll_1 << endl;
		std::cout << "yaw_2:" << yaw_2 << " pitch_2:" << pitch_2 << " roll_2:" << roll_2 << endl;
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
}*/

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
    cout << "eulerAngles ==> Rotation" << endl;
    std::cout << "yaw:" << yaw << " pitch:" << pitch << " roll:" << roll << std::endl;
    //顺规：欧拉角的旋转顺序
    //由欧拉角到旋转矩阵与顺规有关，以下是yaw-->pitch-->roll顺序构造旋转矩阵
    //谁先旋转谁的矩阵在右边
    Matrix3d R_pitch = pitch_to_R(pitch);
    Matrix3d R_yaw = yaw_to_R(yaw);
    Matrix3d R_roll = roll_to_R(roll);
    Matrix3d R_ypr = R_roll * R_pitch * R_yaw;
    std::cout << "R_yaw:" << endl;
    std::cout << R_yaw << endl;
    std::cout << "R_ypr:" << endl;
    std::cout << R_ypr << std::endl;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    std::cout << "AngleAxisd yaw_Angle:" << endl;
    std::cout << yawAngle.matrix() << endl;
    std::cout << "AngleAxisd得到的旋转矩阵与R_x公式是一样的" << endl;
    
    //四元数转换顺序矩阵相乘顺序不变
    Quaterniond q = rollAngle  * pitchAngle * yawAngle;
    Matrix3d R_result = q.toRotationMatrix();
    std::cout << "AngleAxisd R_ypr:" << R_result << std::endl;
    return R_result;
}

int main()
{


    double roll = 0.028468914;
    //double pitch = 0.023796266;
    double pitch = M_PI / 2;
    double yaw = -0.072162271;

    std::cout << "roll:" << roll << " pitch:"
    << pitch << " yaw:" << yaw << std::endl;

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
    Matrix3d R_rpy = R_yaw * R_pitch * R_roll;
    std::cout << "旋转矩阵:" << endl;
    std::cout << R_rpy << endl;
    std::cout << "已知顺规roll-pitch-yaw， 反向求解欧拉角" << std::endl;
    Vector3d ypr = R2rpy(R_rpy);
    std::cout << "yaw:"<< ypr[0] << " pitch:" << ypr[1] << " roll:" << ypr[2] << std::endl;
    
    // yaw->pitch->roll 顺序 
    std::cout << "已知顺规yaw-pitch-roll， 构造旋转矩阵，先旋转的在右边" << std::endl;
    Matrix3d R_ypr = R_roll * R_pitch * R_yaw;
    std::cout << "旋转矩阵:" << endl;
    std::cout << R_ypr << endl;
    std::cout << "已知顺规yaw-pitch-roll， 反向求解欧拉角" << std::endl;
    ypr = R2ypr(R_ypr);
    std::cout << "yaw:"<< ypr[0] << " pitch:" << ypr[1] << " roll:" << ypr[2] << std::endl;

    std::cout << "已知旋转矩阵R，反向求解欧拉角:" << std::endl;
    Matrix3d R;
    R << 0,0,1,
          -1, 0, 0,
          0,-1,0;
    std::cout << R << endl;
    ypr = R2ypr(R);
    std::cout << "yaw--pitch-roll顺规:求解得到" << endl;
    std::cout << "yaw:"<< ypr[0] << " pitch:" << ypr[1] << " roll:" << ypr[2] << std::endl;
    ypr = R2rpy(R);
    std::cout << "roll--pitch-yaw顺规:求解得到" << endl;
    std::cout << "yaw:"<< ypr[0] << " pitch:" << ypr[1] << " roll:" << ypr[2] << std::endl;

    std::cout << "测试AngelAxisd方式构造旋转矩阵"  << std::endl;
    yaw = 1.5288324;
    pitch = 0.16420551;
    roll = 0.51744276;
    std::cout << "yaw:" << yaw << " pitch:"
    << pitch << " roll:" << roll << std::endl;
    R_ypr = roll_to_R(roll) * pitch_to_R(pitch) * yaw_to_R(yaw);
    std::cout << "已知顺规yaw-pitch-roll， 构造旋转矩阵，先旋转的在右边" << std::endl;
    std::cout << "旋转矩阵:" << endl;
    std::cout << R_ypr << endl;
    std::cout << "已知顺规yaw-pitch-roll， 反向求解欧拉角" << std::endl;
    ypr = R2ypr(R_ypr);
    std::cout << "yaw:"<< ypr[0] << " pitch:" << ypr[1] << " roll:" << ypr[2] << std::endl;
    
      
    R_ypr = eulerAngle_to_R(yaw, pitch, roll);
    std::cout << "AngelAxist方式得到的旋转矩阵:" << endl;
    std::cout << R_ypr << endl;
    


}
