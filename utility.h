#pragma once

class Utility
{
public:
    template<typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase <Derived>& theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion <Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        //dq.w() = static_cast<Scalar_t>(1.0);
        //dq.x() = half_theta.x();
        //dq.y() = half_theta.y();
        //dq.z() = half_theta.z();

        //rad
        //yaw -- pitch --> roll
        double yaw = theta.x();
        double pitch = theta.y();
        double roll = theta.z();

        std::cout << "yaw:" << yaw << " pitch:" << pitch 
            << " roll:" << roll << std::endl;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(pitch), 0., sin(pitch),
            0., 1., 0.,
            -sin(pitch), 0., cos(pitch);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(roll), -sin(roll),
            0., sin(roll), cos(roll);

        dq = Rz * Ry * Rx;

        std::cout << "dq-->" << "x:"<< dq.x() << " y:" << dq.y() 
            <<" z:" << dq.z() << " w:" << dq.w() << std::endl;

        return dq;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase <Derived>& q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template<typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase <Derived>& q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase <Derived>& q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w()
            * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase <Derived>& p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w()
            * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft_JPL(const Eigen::QuaternionBase <Derived>& q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(3, 3) = qq.w();

        ans.template block<3, 3>(0, 0) =
            qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());

        ans.template block<3, 1>(0, 3) = qq.vec();
        ans.template block<1, 3>(3, 0) = -qq.vec().transpose();
        return ans;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright_JPL(const Eigen::QuaternionBase <Derived>& p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(3, 3) = pp.w();

        ans.template block<3, 3>(0, 0) =
            pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        ans.template block<3, 1>(0, 3) = pp.vec();
        ans.template block<1, 3>(3, 0) = -pp.vec().transpose();

        return ans;
    }

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R)
    {
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

        return ypr / M_PI * 180.0;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> R2ypr_t(const Eigen::MatrixBase <Derived>& R)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Matrix<Scalar_t, 3, 1> n = R.col(0);
        Eigen::Matrix<Scalar_t, 3, 1> o = R.col(1);
        Eigen::Matrix<Scalar_t, 3, 1> a = R.col(2);

        Eigen::Matrix<Scalar_t, 3, 1> ypr;
        Scalar_t y = atan2(n(1), n(0));
        Scalar_t p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        Scalar_t r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / Scalar_t(M_PI) * Scalar_t(180.0);
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase <Derived>& ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d& g)
    {
        Eigen::Matrix3d R0;
        Eigen::Vector3d ng1 = g.normalized();
        Eigen::Vector3d ng2 = Eigen::Vector3d(0, 0, 1.0);
        R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
        double yaw = Utility::R2ypr(R0).x();

        R0 = Utility::ypr2R(Eigen::Vector3d(-yaw, 0, 0)) * R0;
        return R0;
    }

    template<typename T>
    static T normalizeAngle(const T& angle_degrees)
    {
        T two_pi(2.0 * 180);
        if (angle_degrees > 0)
        {
            return angle_degrees -
                two_pi * std::floor((angle_degrees + T(180)) / two_pi);
        }
        else
        {
            return angle_degrees +
                two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
        }
    };

    static Eigen::Vector3d quaternion2euler_zyx(const Eigen::Quaterniond & q)
    {
        Eigen::Vector3d euler;
        euler(0) = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
                         1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        euler(1) = asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
        euler(2) = atan2(2.0 *(q.x() * q.y() + q.w() * q.z()),
                         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        return euler;
    }
};