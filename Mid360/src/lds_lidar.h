
#include <memory>
#include <mutex>
#include <vector>

#include "lds.h"
#include "comm/comm.h"


#include "rapidjson/document.h"
namespace livox_mid {
class LdsLidar : public Lds
{
    public:
        static LdsLidar *GetInstance(double publish_freq) {
            //    printf("LdsLidar *GetInstance\n");
            static LdsLidar lds_lidar(publish_freq);
            return &lds_lidar;
        }

        bool InitLdsLidar(const std::string& path_name);
        bool Start();
        bool isInitialized(){return is_initialized_;}
        int DeInitLdsLidar(void);
        bool InitLidars(const std::string& path_name);

    private:
        LdsLidar(double publish_freq);
        LdsLidar(const LdsLidar &) = delete;
        ~LdsLidar();
        LdsLidar &operator=(const LdsLidar &) = delete;

        bool ParseSummaryConfig(const std::string& path_name);

        bool InitLivoxLidar(const std::string& path_name);    // for new SDK

        bool LivoxLidarStart();

        void ResetLdsLidar(void);

        void SetLidarPubHandle();

        // auto connect mode
        void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
        void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
        bool IsAutoConnectMode(void) { return auto_connect_mode_; }

        virtual void PrepareExit(void);

    public:
        std::mutex config_mutex_;

    private:
        std::string path_;
        LidarSummaryInfo lidar_summary_info_;

        bool auto_connect_mode_;
        uint32_t whitelist_count_;
        volatile bool is_initialized_;
        char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];
};

}  // namespace livox_ros





