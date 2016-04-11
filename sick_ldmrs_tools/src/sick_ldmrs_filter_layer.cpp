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

ros::Publisher pub_layer1;
ros::Publisher pub_first;
ros::Publisher pub_last;

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{

  pcl::PointCloud<sick_ldmrs_msgs::SICK_LDMRS_Point>::Ptr cloud(new pcl::PointCloud<sick_ldmrs_msgs::SICK_LDMRS_Point>);
  pcl::fromROSMsg(*pc, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

  // layer1: only publish points from second layer
  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (cloud->points[i].layer == 1)
    {
      cloud_filtered->points.push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }
  }

  sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_filtered, *msg);
  msg->header = pc->header;
  pub_layer1.publish(msg);

  // first: only publish first echo
  cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (cloud->points[i].echo == 0)
    {
      cloud_filtered->points.push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }
  }

  msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_filtered, *msg);
  msg->header = pc->header;
  pub_first.publish(msg);

  // last: only publish last echo
  cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (!(cloud->points[i].flags & sick_ldmrs_msgs::FlagTransparent))
    {
      cloud_filtered->points.push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }
  }

  msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_filtered, *msg);
  msg->header = pc->header;
  pub_last.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs_filter_layer");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cloud", 1, callback);
  pub_layer1 = nh.advertise<sensor_msgs::PointCloud2>("layer1", 1);
  pub_first = nh.advertise<sensor_msgs::PointCloud2>("first", 1);
  pub_last = nh.advertise<sensor_msgs::PointCloud2>("last", 1);

  ros::spin();

  return 0;
}
