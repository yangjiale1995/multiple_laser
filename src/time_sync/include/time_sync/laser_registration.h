#ifndef LASER_REGISTRATION_H
#define LASER_REGISTRATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define N_SCANS 16
#define SCAN_PERIOD 0.1
#define IMU_FREQUENCY 200

/*
 * 点云信息：转换矩阵，时间戳，点云
 * */
struct LaserInfo
{
    Eigen::Matrix4f laser_transform_;
    double laser_timestamp_;
    pcl::PointCloud<pcl::PointXYZI> laser_;
};

/*
 * IMU信息：时间戳，roll, pitch, yaw, 加速度，速度，位移
 * */
class ImuInfo
{
public:
    //构造函数
    ImuInfo();
    //析构函数
    ~ImuInfo();
    //IMU信息赋值
    void setImuInfo(double timestamp, double roll, double pitch, double yaw, double accx, double accy, double accz);
    //读取IMU信息
    void getImuInfo(double timestamp, double &roll, double &pitch, double &yaw, double &shiftx, double &shifty, double &shiftz);
private:
    std::vector<double> imu_timestamp_;

    std::vector<double> imu_roll_;
    std::vector<double> imu_pitch_;
    std::vector<double> imu_yaw_;

    std::vector<double> imu_accx_;
    std::vector<double> imu_accy_;
    std::vector<double> imu_accz_;
    
    std::vector<double> imu_velox_;
    std::vector<double> imu_veloy_;
    std::vector<double> imu_veloz_;
    
    std::vector<double> imu_shiftx_;
    std::vector<double> imu_shifty_;
    std::vector<double> imu_shiftz_;

    int imu_index_;     //下标
};


/*
 * 点云注册：imu信息，点云信息
 * */
class LaserRegistration
{
public:
    //构造函数
    LaserRegistration(int laser_num);
    //析构函数
    ~LaserRegistration();
    
    //赋值点云
    void setPointCloud(int index, pcl::PointCloud<pcl::PointXYZI> laser);
    //旋转矩阵
    void setLaserMatrix(int index, Eigen::Matrix4f guess);
    
    void setLaserTimestamp(int index, double timestamp);
    //IMU信息
    void setImuInfo(double timestamp, double roll, double pitch, double yaw, double accx, double accy, double accz);
    //最终拼接结果
    void getFinalLaser(pcl::PointCloud<pcl::PointXYZI> &laser);

private:

    ImuInfo imu_;
    std::vector<LaserInfo> lasers_;
    
    double timestamp_;
};

#endif

