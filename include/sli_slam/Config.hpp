#ifndef SLI_SLAM_CONFIG_HPP
#define SLI_SLAM_CONFIG_HPP

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

namespace sli_slam{

// Config class
// Singleton pattern
class Config {
public:

    ~Config();

    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T>
    static T Get(const std::string &key) {
        return static_cast<T>(config_->file_[key]);
    }
    
private:

    Config() = default;

    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

};

} // namespace sli_slam

#endif