//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"


#include <inttypes.h>
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
// #include "include/ros_headers.h"

// #include "driver_node.h"
#include "lds_lidar.h"
namespace livox_mid
{

    /** Lidar Data Distribute Control--------------------------------------------*/

    Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
               double frq, std::string &frame_id, bool lidar_bag, bool imu_bag, double max_length, std::string ip)
        : transfer_format_(format),
          use_multi_topic_(multi_topic),
          data_src_(data_src),
          output_type_(output_type),
          publish_frq_(frq),
          frame_id_(frame_id),
          enable_lidar_bag_(lidar_bag),
          enable_imu_bag_(imu_bag),
          max_length_(max_length),
          lidar_ip_(ip)
    {
        publish_period_ns_ = kNsPerSecond / publish_frq_;
        lds_ = nullptr;
        //        memset(private_pub_, 0, sizeof(private_pub_));
        //        memset(private_imu_pub_, 0, sizeof(private_imu_pub_));
        //        global_pub_ = nullptr;
        //        global_imu_pub_ = nullptr;
        //        cur_node_ = nullptr;
        //        bag_ = nullptr;
    }

    Lddc::~Lddc()
    {

        PrepareExit();
    }

    int Lddc::RegisterLds(Lds *lds)
    {
        if (lds_ == nullptr)
        {
            lds_ = lds;
            return 0;
        }
        else
        {
            return -1;
        }
    }

    void Lddc::DistributePointCloudData(std::vector<PointXyzlt> &get_cloud,uint64_t &timestamp)
    {
        if (!lds_)
        {
            std::cout << "lds is not registered" << std::endl;
        }
        if (lds_->IsRequestExit())
        {
            // std::cout << "DistributePointCloudData is RequestExit" << std::endl;
        }
        //        LogInfo("wait pcd cloud");
        //        lds_->pcd_semaphore_.Wait();
        //        LogInfo("count: "<<uint32_t(lds_->lidar_count_));
        // uint32_t currentHandle = IpStringToNum(lidar_ip_);
        //        LogInfo("handle: "<<currentHandle);
        for (uint32_t i = 0; i < lds_->lidar_count_; i++)
        {
            uint32_t lidar_id = i;
            LidarDevice *lidar = &lds_->lidars_[lidar_id];
            LidarDataQueue *p_queue = &lidar->data;
            // std::cout << "lidar handle: " << lidar->handle << " " << lidar->connect_state << " " << p_queue->size << std::endl;
            if ((kConnectStateSampling != lidar->connect_state) || p_queue == nullptr)
            {
                continue;
            }
            //            LogInfo("handle: "<<currentHandle<<" "<<lidar->connect_state);
            PollingLidarPointCloudData(lidar_id, lidar, get_cloud,timestamp);
     
        }
    }

    void Lddc::DistributeImuData(ImuData& get_imu)
    {
        if (!lds_)
        {
            std::cout << "lds is not registered" << std::endl;
        }
        if (lds_->IsRequestExit())
        {
            std::cout << "DistributeImuData is RequestExit" << std::endl;
        }

        //    lds_->imu_semaphore_.Wait();
        for (uint32_t i = 0; i < lds_->lidar_count_; i++)
        {
            uint32_t lidar_id = i;
            LidarDevice *lidar = &lds_->lidars_[lidar_id];
            LidarImuDataQueue *p_queue = &lidar->imu_data;
            if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr))
            {
                continue;
            }
            PollingLidarImuData(lidar_id, lidar, get_imu);
            // if (sqrt(get_imu.acc_x * get_imu.acc_x + get_imu.acc_y * get_imu.acc_y + get_imu.acc_z * get_imu.acc_z) >= 9.8)
        }
    }

    void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar, std::vector<PointXyzlt> &get_cloud, uint64_t &timestamp)
    {
        LidarDataQueue *p_queue = &lidar->data;
        if (p_queue == nullptr || p_queue->storage_packet == nullptr)
        {
            return;
        }
        while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue))
        {
            //            if (kPointCloud2Msg == transfer_format_) {
            //                PublishPointcloud2(p_queue, index);
            //            } else if (kLivoxCustomMsg == transfer_format_) {
            //                PublishCustomPointcloud(p_queue, index);
            //            } else if (kPclPxyziMsg == transfer_format_) {
            //                PublishPclMsg(p_queue, index);
            //            }
            PublishPclMsg(p_queue, index, get_cloud,timestamp);
        }
    }

    void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar, ImuData &get_imu)
    {
        LidarImuDataQueue &p_queue = lidar->imu_data;
        if (!lds_->IsRequestExit() && !p_queue.Empty())
        {
            PublishImuData(p_queue, index, get_imu);           
        }
    }

    void Lddc::PrepareExit(void)
    {
        if (lds_)
        {
            lds_->PrepareExit();
            lds_ = nullptr;
        }
    }

    //    void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) {
    //        while(!QueueIsEmpty(queue)) {
    //            StoragePacket pkg;
    //            QueuePop(queue, &pkg);
    //            if (pkg.points.empty()) {
    //                printf("Publish point cloud2 failed, the pkg points is empty.\n");
    //                continue;
    //            }

    //            PointCloud2 cloud;
    //            uint64_t timestamp = 0;
    //            InitPointcloud2Msg(pkg, cloud, timestamp);
    //            PublishPointcloud2Data(index, timestamp, cloud);
    //        }
    //    }

    //    void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
    //        while(!QueueIsEmpty(queue)) {
    //            StoragePacket pkg;
    //            QueuePop(queue, &pkg);
    //            if (pkg.points.empty()) {
    //                printf("Publish custom point cloud failed, the pkg points is empty.\n");
    //                continue;
    //            }

    //            CustomMsg livox_msg;
    //            InitCustomMsg(livox_msg, pkg, index);
    //            FillPointsToCustomMsg(livox_msg, pkg);
    //            PublishCustomPointData(livox_msg, index);
    //        }
    //    }

    /* for pcl::pxyzi */
    void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index, std::vector<PointXyzlt> &get_cloud, uint64_t &timestamp)
    {
        while (!QueueIsEmpty(queue))
        {
            StoragePacket pkg;
            QueuePop(queue, &pkg);
            if (pkg.points.empty())
            {
                continue;
            }

            // PointCloud cloud;
            // uint64_t timestamp = 0;
            if (!pkg.points.empty())
            {
                timestamp = pkg.base_time + pkg.points[0].offset_time;
                get_cloud = pkg.points;
            }
            // InitPclMsg(pkg, cloud, timestamp);
            // FillPointsToPclMsg(pkg, get_cloud);
            // *get_cloud += cloud;
            //            PublishPclData(index, timestamp, cloud);
            //            pcl::copyPointCloud(cloud, *get_cloud);
            // std::cout << "PointCloud:" << timestamp << std::endl;
        }
        return;
    }

    void Lddc::InitPclMsg(const StoragePacket &pkg, PointCloud &cloud, uint64_t &timestamp)
    {
        // cloud.header.frame_id.assign(frame_id_);
        // cloud.height = 1;
        // cloud.width = pkg.points_num;

        // if (!pkg.points.empty())
        // {
        //     timestamp = pkg.base_time + pkg.points[0].offset_time;
        // }
        // cloud.header.stamp = timestamp / 1000.0; // to pcl ros time stamp
        return;
    }

    void Lddc::FillPointsToPclMsg(const StoragePacket &pkg, std::vector<PointXyzlt> &pcl_msg)
    {
        // if (pkg.points.empty())
        // {
        //     return;
        // }

        // uint32_t points_num = pkg.points_num;
        // const std::vector<PointXyzlt> &points = pkg.points;
        // //        LogInfo("points_num: "<<points_num);
        // int prev=points[0].offset_time;
        // for (uint32_t i = 0; i < points_num; ++i)
        // {
        //     pcl::PointXYZ point;
        //     if (points[i].offset_time < prev){
        //         // std::cout << i << std::endl;
        //         std::cout << "time disordered !:" << i << " " << points[i].offset_time << " " << prev << std::endl;
        //     }
        //     prev = points[i].offset_time;
        //     if ((points[i].x * points[i].x + points[i].y * points[i].y + points[i].z * points[i].z) > max_length_ * max_length_)
        //         continue;
        //     point.x = -points[i].y;
        //     point.y = -points[i].z;
        //     point.z = points[i].x;
        //     //            point.x = points[i].x;
        //     //            point.y = points[i].y;
        //     //            point.z = points[i].z;
        //     //            point.intensity = points[i].intensity;

        //     pcl_msg.points.push_back(std::move(point));
        // }
        return;
    }

    void Lddc::PublishImuData(LidarImuDataQueue &imu_data_queue, const uint8_t index, ImuData &get_imu)
    {
        ImuData imu_data;
        if (!imu_data_queue.Pop(imu_data))
        {
            // printf("Publish imu data failed, imu data queue pop failed.\n");
            return;
        }

        // sensor_msgs::Imu imu_msg;
        uint64_t timestamp;
        // InitImuMsg(imu_data, imu_msg, timestamp);
        get_imu.time_stamp = imu_data.time_stamp;
        //mid360的重力加速度单位为g，这里把单位换成m/s^2
        get_imu.acc_x = imu_data.acc_x * 9.7964;
        get_imu.acc_y = imu_data.acc_y * 9.7964;
        get_imu.acc_z = imu_data.acc_z * 9.7964;
        // std::cout << "ImuData:" << imu_data.time_stamp << std::endl;
        // std::cout << "ImuData:" << imu_data.time_stamp << std::endl;
    }

    // void Lddc::CreateBagFile(const std::string &file_name)
    // {
    //     if (!bag_)
    //     {
    //         bag_ = new rosbag::Bag;
    //         bag_->open(file_name, rosbag::bagmode::Write);
    //         DRIVER_INFO(*cur_node_, "Create bag file :%s!", file_name.c_str());
    //     }
    // }

} // namespace livox_mid
