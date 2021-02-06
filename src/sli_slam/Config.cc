#include <string>
#include <memory>

#include <glog/logging.h>

#include "sli_slam/Config.hpp"

using std::string;
using std::shared_ptr;

using cv::FileStorage;

using sli_slam::Config;

shared_ptr<Config> Config::config_ = nullptr;

Config::~Config(){
    if(file_.isOpened()){
        file_.release();
    }
}

bool Config::SetParameterFile(const string &filename){
    if(config_ == nullptr){
        config_ = shared_ptr<Config>(new Config);
    }
    
    config_->file_ = FileStorage(filename.c_str(), FileStorage::READ);

    if(config_->file_.isOpened() == false){
        LOG(ERROR) << "Parameter file " << filename << " fail to load.";
        config_->file_.release();
        return false;
    }
    
    return true;
}