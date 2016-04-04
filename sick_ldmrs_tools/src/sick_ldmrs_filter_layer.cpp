/*
 * Copyright (C) 2016, DFKI GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <ros@jochen.sprickerhof.de>
 *
 */

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sick_ldmrs_msgs/sick_ldmrs_point_type.h>
#include <pcl/point_cloud.h>

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{

  pcl::PointCloud<sick_ldmrs_msgs::SICK_LDMRS_Point>::Ptr cloud(new pcl::PointCloud<sick_ldmrs_msgs::SICK_LDMRS_Point>);
  pcl::fromROSMsg(*pc, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (cloud->points[i].layer == 1)
    {
      cloud_filtered->points.push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud_filtered, msg);
  msg.header = pc->header;
  pub.publish(msg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs_filter_layer");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cloud", 1, callback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);

  ros::spin();

  return 0;
}
