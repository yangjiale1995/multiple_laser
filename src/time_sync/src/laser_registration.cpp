#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <float.h>
#include "laser_registration.h"

//构造函数
ImuInfo::ImuInfo()
{
    //时间戳
    imu_timestamp_.resize(IMU_FREQUENCY);

    //加速度
    imu_accx_.resize(IMU_FREQUENCY);
    imu_accy_.resize(IMU_FREQUENCY);
    imu_accz_.resize(IMU_FREQUENCY);

    //速度
    imu_velox_.resize(IMU_FREQUENCY);
    imu_veloy_.resize(IMU_FREQUENCY);
    imu_veloz_.resize(IMU_FREQUENCY);

    //位移
    imu_shiftx_.resize(IMU_FREQUENCY);
    imu_shifty_.resize(IMU_FREQUENCY);
    imu_shiftz_.resize(IMU_FREQUENCY);

    //下标
    imu_index_ = -1;   //当前imu数据下标
}

//析构函数
ImuInfo::~ImuInfo()
{}

//设置imu信息
void ImuInfo::setImuInfo(double timestamp, double roll, double pitch, double yaw, double accx, double accy, double accz)
{
    //车体坐标系下的加速度计算
    //绕x轴逆时针旋转
    double x1 = accx; 
    double y1 = cos(roll) * accy + sin(roll) * accz;
    double z1 = -sin(roll) * accy + cos(roll) * accz;

    //绕y轴逆时针旋转
    double x2 = cos(pitch) * x1 - sin(pitch) * z1;
    double y2 = y1;
    double z2 = sin(pitch) * x1 + cos(pitch) * z1;
    
    //绕z轴逆时针旋转
    double x3 = cos(yaw) * x2 + sin(yaw) * y2;
    double y3 = -sin(yaw) * x2 + cos(yaw) * y2;
    double z3 = z2;

    //剔除重力
    double accx_g = x3;
    double accy_g = y3;
    double accz_g = z3 - 9.81;

    //下标计算
    imu_index_ = (imu_index_ + 1) % IMU_FREQUENCY;

    //时间戳
    imu_timestamp_[imu_index_] = timestamp;

    // roll pitch yaw
    imu_roll_[imu_index_] = roll;
    imu_pitch_[imu_index_] = pitch;
    imu_yaw_[imu_index_] = yaw;

    // 加速度
    imu_accx_[imu_index_] = accx_g;
    imu_accy_[imu_index_] = accy_g;
    imu_accz_[imu_index_] = accz_g;
    

    int front_index = (imu_index_ - 1 + IMU_FREQUENCY) / IMU_FREQUENCY;    //上一个imu下标
    double time_diff = imu_timestamp_[imu_index_] - imu_timestamp_[front_index];   //时间差

    //位移(匀加速模型)
    imu_shiftx_[imu_index_] = imu_shiftx_[front_index] + imu_velox_[front_index] * time_diff + accx_g * time_diff * time_diff / 2.0;
    imu_shifty_[imu_index_] = imu_shifty_[front_index] + imu_veloy_[front_index] * time_diff + accy_g * time_diff * time_diff / 2.0;
    imu_shiftz_[imu_index_] = imu_shiftz_[front_index] + imu_veloz_[front_index] * time_diff + accz_g * time_diff * time_diff / 2.0;

    //速度(匀加速模型)
    imu_velox_[imu_index_] = imu_velox_[front_index] + accx_g * time_diff;
    imu_veloy_[imu_index_] = imu_veloy_[front_index] + accy_g * time_diff;
    imu_veloz_[imu_index_] = imu_veloz_[front_index] + accz_g * time_diff;
}

//获取IMU信息
void ImuInfo::getImuInfo(double timestamp, double &roll, double &pitch, double &yaw, double &shiftx, double &shifty, double &shiftz)
{
    int imu_point_front = 0;
    //IMU有数值
    if(imu_index_ >= 0)
    {
        //找指定时间戳的IMU信息
        while(imu_point_front != imu_index_)
        {
            if(imu_timestamp_[imu_point_front] > timestamp)
            {
                break;
            }
            imu_point_front = (imu_point_front + 1) % IMU_FREQUENCY;
        }

        //没找到，用最新的IMU信息
        if(timestamp > imu_timestamp_[imu_point_front])
        {
            roll = imu_roll_[imu_point_front];
            pitch = imu_pitch_[imu_point_front];
            yaw = imu_yaw_[imu_point_front];

            //velox = imu_velox_[imu_point_front];
            //veloy = imu_veloy_[imu_point_front];
            //veloz = imu_veloz_[imu_point_front];

            shiftx = imu_shiftx_[imu_point_front];
            shifty = imu_shifty_[imu_point_front];
            shiftz = imu_shiftz_[imu_point_front];
        }
        else        //找到，进行插值
        {
            int imu_point_back = (imu_point_front - 1 + IMU_FREQUENCY) % IMU_FREQUENCY;
            
            float ratio_front = (timestamp - imu_timestamp_[imu_point_back]) / (imu_timestamp_[imu_point_front] - imu_timestamp_[imu_point_back]); 
            float ratio_back = (imu_timestamp_[imu_point_front] - timestamp) / (imu_timestamp_[imu_point_front] - imu_timestamp_[imu_point_back]);

            roll = imu_roll_[imu_point_front] * ratio_front + imu_roll_[imu_point_back] * ratio_back;
            pitch = imu_pitch_[imu_point_front] * ratio_front + imu_pitch_[imu_point_back] * ratio_back;
            if(imu_yaw_[imu_point_front] - imu_yaw_[imu_point_back] > M_PI)
            {
                yaw = imu_yaw_[imu_point_front] * ratio_front + (imu_yaw_[imu_point_back] + 2 * M_PI) * ratio_back;
            }
            else if(imu_yaw_[imu_point_front] - imu_yaw_[imu_point_back] < -M_PI)
            {
                yaw = imu_yaw_[imu_point_front] * ratio_front + (imu_yaw_[imu_point_back] - 2 * M_PI) * ratio_back;
            }
            else
            {
                yaw = imu_yaw_[imu_point_front] * ratio_front + imu_yaw_[imu_point_back] * ratio_back;
            }

            //velox = imu_velox_[imu_point_front] * ratio_front + imu_velox_[imu_point_back] * ratio_back;
            //veloy = imu_veloy_[imu_point_front] * ratio_front + imu_veloy_[imu_point_back] * ratio_back;
            //veloz = imu_veloz_[imu_point_front] * ratio_front + imu_veloz_[imu_point_back] * ratio_back;

            shiftx = imu_shiftx_[imu_point_front] * ratio_front + imu_shiftx_[imu_point_back] * ratio_back;
            shifty = imu_shifty_[imu_point_front] * ratio_front + imu_shifty_[imu_point_back] * ratio_back;
            shiftz = imu_shiftz_[imu_point_front] * ratio_front + imu_shiftz_[imu_point_back] * ratio_back;
        }
    }
    else        //没有IMU信息，全部置0
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
        
        //velox = 0;
        //veloy = 0;
        //veloz = 0;

        shiftx = 0;
        shifty = 0;
        shiftz = 0;
    }
}

//构造函数
//laser_num 雷达个数
LaserRegistration::LaserRegistration(int laser_num)
{
    lasers_.resize(laser_num);
}

//设置IMU信息
void LaserRegistration::setImuInfo(double timestamp, double roll, double pitch, double yaw, double accx, double accy, double accz)
{
    imu_.setImuInfo(timestamp, roll, pitch, yaw, accx, accy, accz);
}

//设置点云
//callback使用
void LaserRegistration::setPointCloud(int index, pcl::PointCloud<pcl::PointXYZI> laser)
{
    lasers_[index].laser_ = laser;
}

//设置转换矩阵
//初始化时设置的参数
void LaserRegistration::setLaserMatrix(int index, Eigen::Matrix4f guess)
{
    lasers_[index].laser_transform_ = guess;
}

void LaserRegistration::setLaserTimestamp(int index, double timestamp)
{
    lasers_[index].laser_timestamp_ = timestamp;
}

//析构函数
LaserRegistration::~LaserRegistration()
{}

//点云
void LaserRegistration::getFinalLaser(pcl::PointCloud<pcl::PointXYZI> &laser)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI> > scan(N_SCANS);
    timestamp_ = DBL_MAX;
    for(int i = 0; i < lasers_.size(); i ++)
    {
        timestamp_ = timestamp_ < lasers_[i].laser_timestamp_ ? timestamp_ : lasers_[i].laser_timestamp_;
        pcl::transformPointCloud(lasers_[i].laser_, lasers_[i].laser_, lasers_[i].laser_transform_);
    }
    
    //imu_.getImuInfo(timestamp_, roll_, pitch_, yaw_, shiftx_, shifty_, shiftz_);

    for(int i = 0; i < lasers_.size(); i ++)
    {
        //赋值
        double time1 = lasers_[i].laser_timestamp_;
        pcl::PointCloud<pcl::PointXYZI> laseri = lasers_[i].laser_;
        int cloudSize = laseri.points.size();
        
        float start_ori = atan2(laseri.points[0].y, laseri.points[0].x); 
        float end_ori = atan2(laseri.points[cloudSize - 1].y, laseri.points[cloudSize - 1].x) + 2 * M_PI;
        if(end_ori - start_ori > 3 * M_PI)
        {
            end_ori -= 2 * M_PI;
        }
        else if(end_ori - start_ori < M_PI)
        {
            end_ori += 2 * M_PI;
        }


        bool halfPassed = false;
        for(int j = 0; j < cloudSize; j ++)
        {
            pcl::PointXYZI point = laseri.points[j];
            int scan_id = round(atan2(point.z, sqrt(pow(point.x, 2) + pow(point.y, 2))) * 180.0 / M_PI);
            if(scan_id < 0)
            {
                scan_id += (N_SCANS);
            }
            if(scan_id < 0 || scan_id > N_SCANS)
            {
                continue;
            }
            point.intensity = scan_id;

            float cur_ori = atan2(point.y, point.x);
            if(!halfPassed)
            {
                if(cur_ori < start_ori - M_PI / 2.0)
                {
                    cur_ori += 2 * M_PI;
                }
                else if(cur_ori > start_ori + M_PI * 3.0 / 2.0)
                {
                    cur_ori -= 2 * M_PI;
                }
                if(cur_ori - start_ori > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                cur_ori += 2 * M_PI;

                if(cur_ori < end_ori - M_PI * 3.0 / 2.0)
                {
                    cur_ori += 2 * M_PI;
                }
                else if(cur_ori > end_ori + M_PI / 2.0)
                {
                    cur_ori -= 2 * M_PI;
                }
            }

            double point_time = time1 + (cur_ori - start_ori) / (end_ori - start_ori) * SCAN_PERIOD;
            
            double cur_roll, cur_pitch, cur_yaw;
            double cur_shiftx, cur_shifty, cur_shiftz;
            imu_.getImuInfo(point_time, cur_roll, cur_pitch, cur_yaw, cur_shiftx, cur_shifty, cur_shiftz);
            
            //绕x轴逆时针旋转
            float x1 = point.x - cur_shiftx;
            float y1 = cos(cur_roll) * (point.y - cur_shifty) + sin(cur_roll) * (point.z - cur_shiftz);
            float z1 = -sin(cur_roll) * (point.y - cur_shifty) + cos(cur_roll) * (point.z - cur_shiftz);

            //绕y轴逆时针旋转
            float x2 = cos(cur_pitch) * x1 - sin(cur_pitch) * z1;
            float y2 = y1;
            float z2 = sin(cur_pitch) * x1 + cos(cur_pitch) * z1;

            float x3 = cos(cur_yaw) * x2 + sin(cur_yaw) * y2;
            float y3 = -sin(cur_yaw) * x2 + cos(cur_yaw) * y2;
            float z3 = z2;

            point.x = x3;
            point.y = y3;
            point.z = z3;

            scan[scan_id].push_back(point);
        }
    }

    for(int i = 0; i < N_SCANS; i ++)
    {
        laser += scan[i];
    }
}


