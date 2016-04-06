#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace sick_ldmrs_msgs
{

struct SICK_LDMRS_Point
{
  PCL_ADD_POINT4D;
  uint16_t echowidth;
  uint8_t layer;
  uint8_t echo;
  uint8_t flags;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

enum Flags
{
  FlagTransparent = 0x01,    // set if at least one more echo behind this scan point
  FlagClutter     = 0x02,    // set if scan point is classified as atmospheric noise such as rain, dust, or similar
  FlagGround      = 0x04,    // set if scan point is classified as ground (only available if corresponding processing is active)
  FlagDirt        = 0x08     // set if scan point is classified as dirt (usually dirty front screen of sensor housing)
};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(sick_ldmrs_msgs::SICK_LDMRS_Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint16_t, echowidth, echowidth)
                                  (uint8_t, layer, layer)
                                  (uint8_t, echo, echo)
                                  (uint8_t, flags, flags)
                                 )
