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
