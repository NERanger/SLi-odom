#ifndef SLI_SLAM_MAPPOINT_HPP
#define SLI_SLAM_MAPPOINT_HPP

#include <memory>
#include <mutex>
#include <list>

#include "sli_slam/Common.hpp"

namespace sli_slam{

class Feature;

class MapPoint{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<MapPoint> Ptr;

    MapPoint() = default;

    MapPoint(unsigned long id, Vec3 position) : id_(id), position_(position){}

    Vec3 Position();  // Postion in world frame
    void SetPosition(const Vec3 &position);
    void AddObservation(Feature::Ptr feat);
    void RemoveObservation(Feature::Ptr feat);
    
    unsigned long Id() const {return id_;}
    int ObservedTimes() const {return observed_times_;}

    static MapPoint::Ptr CreateNewMapPoint();

private:
    bool is_outlier_ = false;
    unsigned long id_ = 0;

    // times for being observed by feature matching algorithm
    int observed_times_ = 0;

    // Postion in world frame
    Vec3 position_ = Vec3::Zero();

    std::mutex data_mutex_;

    std::list<std::weak_ptr<Feature>> observations_;
};

}  // namespace sli_slam

#endif