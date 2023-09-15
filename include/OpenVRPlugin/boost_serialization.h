#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/boost_unordered_map.hpp>
#include <boost/serialization/string.hpp>
#include <openvr/openvr.h>

namespace boost
{
namespace serialization
{

template<class Archive>
void serialize(Archive & ar, vr::TrackedDevicePose_t & p, const unsigned int version)
{
    ar & p.bDeviceIsConnected;
    ar & p.bPoseIsValid;
    ar & p.eTrackingResult;
    ar & p.mDeviceToAbsoluteTracking.m;
    ar & p.vAngularVelocity.v;
    ar & p.vVelocity.v;
}


};
};