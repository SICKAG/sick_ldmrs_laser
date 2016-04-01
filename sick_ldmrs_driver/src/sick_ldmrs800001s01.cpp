/*
 * Copyright (C) 2015, DFKI GmbH
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
 *  Created on: 20.11.2015
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *
 */

#include <iostream>

#include <pcl_conversions/pcl_conversions.h>

#include "sick_ldmrs_driver/sick_ldmrs800001s01.hpp"

#include <sick_ldmrs/datatypes/EvalCaseResults.hpp>
#include <sick_ldmrs/datatypes/EvalCases.hpp>
#include <sick_ldmrs/datatypes/Fields.hpp>
#include <sick_ldmrs/datatypes/Measurement.hpp>
#include <sick_ldmrs/datatypes/Msg.hpp>
#include <sick_ldmrs/datatypes/Scan.hpp>

#include <sick_ldmrs/devices/LD_MRS.hpp>

#include <sick_ldmrs/tools/errorhandler.hpp>
#include <sick_ldmrs/tools/toolbox.hpp>

#include <sick_ldmrs_msgs/ObjectArray.h>
#include <tf/transform_datatypes.h>


namespace sick_ldmrs_driver
{

SickLDMRS::SickLDMRS(Manager *manager, boost::shared_ptr<diagnostic_updater::Updater> diagnostics)
  : application::BasicApplication()
  , diagnostics_(diagnostics)
  , manager_(manager)
  , expected_frequency_(12.5)
{
  dynamic_reconfigure::Server<SickLDMRSDriverConfig>::CallbackType f;
  f = boost::bind(&SickLDMRS::update_config, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(f);

  // point cloud publisher
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 100);

  object_pub_ = nh_.advertise<sick_ldmrs_msgs::ObjectArray>("objects", 1);

  diagnostics_->setHardwareID("none");   // set from device after connection
  diagnosticPub_ = new diagnostic_updater::DiagnosedPublisher<sensor_msgs::PointCloud2>(pub_, *diagnostics_,
      // frequency should be target +- 10%
      diagnostic_updater::FrequencyStatusParam(&expected_frequency_, &expected_frequency_, 0.1, 10),
      // timestamp delta can be from -1 seconds to 1.3x what it ideally is at the lowest frequency
      diagnostic_updater::TimeStampStatusParam(-1, 1.3 * 1.0 / 12.5));
}

SickLDMRS::~SickLDMRS()
{
  delete diagnosticPub_;
}

void SickLDMRS::setData(BasicData &data)
{
  //
  // Do something with it.
  //
  // Here, we just print what we've got.
  //
  std::string datatypeStr;
  std::string sourceIdStr;

  switch (data.getDatatype())
  {
  case Datatype_Scan:
    datatypeStr = "Scan (" + ::toString(((Scan&)data).getNumPoints()) + " points)";
    {
      Scan* scan = dynamic_cast<Scan*>(&data);
      std::vector<ScannerInfo> scannerInfos = scan->getScannerInfos();
      std::vector<ScannerInfo>::const_iterator it;
      for (it = scannerInfos.begin(); it != scannerInfos.end(); ++it)
      {
        const Time& time = it->getStartTimestamp();
        ROS_INFO("LdmrsApp::setData(): Scan start time: %s (%s)",
                 time.toString().c_str(),
                 time.toLongString().c_str());
      }

      PointCloud::Ptr cloud = boost::make_shared<PointCloud>();
      cloud->header.frame_id = config_.frame_id;
      // not using time stamp from scanner here, because it is delayed by up to 1.5 seconds
      cloud->header.stamp = (ros::Time::now().toSec() - 1 / expected_frequency_) * 1e6;

      cloud->height = 1;
      cloud->width = scan->size();
      for (size_t i = 0; i < scan->size(); ++i)
      {
        const ScanPoint& p = (*scan)[i];
        cloud->points.push_back(pcl::PointXYZ(p.getX(), p.getY(), p.getZ()));
      }

      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*cloud, msg);
      diagnosticPub_->publish(msg);
    }
    break;
  case Datatype_Objects:
    datatypeStr = "Objects (" + ::toString(((ObjectList&)data).size()) + " objects)";
    pubObjects((ObjectList&)data);
    break;
  case Datatype_Fields:
    datatypeStr = "Fields (" + ::toString(((Fields&)data).getFields().size()) + " fields, " +
                  ::toString(((Fields&)data).getNumberOfValidFields()) + " of which are valid)";
    break;
  case Datatype_EvalCases:
    datatypeStr = "EvalCases (" + ::toString(((EvalCases&)data).getEvalCases().size()) + " cases)";
    break;
  case Datatype_EvalCaseResults:
    datatypeStr = "EvalCaseResults (" + ::toString(((EvalCaseResults&)data).size()) + " case results)";
    break;
  case Datatype_Msg:
    datatypeStr = "Msg (" + ((Msg&)data).toString() + ")";
    diagnostics_->broadcast(diagnostic_msgs::DiagnosticStatus::WARN, ((Msg&)data).toString());
    diagnostics_->force_update();
    break;
  case Datatype_MeasurementList:
    datatypeStr = "MeasurementList (" + ::toString(((MeasurementList&)data).m_list.size()) + " entries)";
    break;
  default:
    datatypeStr = "(unknown)";
  }

  sourceIdStr = ::toString(data.getSourceId());

  ROS_INFO("LdmrsApp::setData(): Called with data of type %s from ID %s", datatypeStr.c_str(), sourceIdStr.c_str());
}

void SickLDMRS::validate_config(SickLDMRSDriverConfig &conf)
{
  if (conf.start_angle <= conf.end_angle)
  {
    ROS_WARN("Start angle must be greater than end angle. Adjusting start_angle.");
    conf.start_angle = conf.end_angle;  // TODO: - 2 * ticks2rad
  }

  if (conf.angular_resolution_type != SickLDMRSDriver_ConstantRes
      && conf.scan_frequency != SickLDMRSDriver_ScanFreq1250)
  {
    ROS_WARN("Focused/flexible resolution only available at 12.5 Hz scan frequency. Adjusting scan frequency.");
    conf.scan_frequency = SickLDMRSDriver_ScanFreq1250;
  }

  if (conf.ignore_near_range && conf.layer_range_reduction != SickLDMRSDriver_RangeLowerReduced)
  {
    ROS_WARN("If ignore_near_range is set, layer_range_reduction must be set to 'Lower 4 layers reduced range'. Adjusting layer_range_reduction.");
    conf.layer_range_reduction = SickLDMRSDriver_RangeLowerReduced;
  }
}

void SickLDMRS::pubObjects(datatypes::ObjectList &objects)
{
  sick_ldmrs_msgs::ObjectArray oa;
  oa.header.frame_id = config_.frame_id;
  // not using time stamp from scanner here, because it is delayed by up to 1.5 seconds
  oa.header.stamp = ros::Time::now();
  oa.objects.resize(objects.size());

  for (int i = 0; i < objects.size(); i++)
  {
    oa.objects[i].id = objects[i].getObjectId();
    oa.objects[i].tracking_time = ros::Time::now() - ros::Duration(objects[i].getObjectAge() / expected_frequency_);
    oa.objects[i].last_seen = ros::Time::now() - ros::Duration(objects[i].getHiddenStatusAge() / expected_frequency_);
    oa.objects[i].velocity.twist.linear.x = objects[i].getAbsoluteVelocity().getX();
    oa.objects[i].velocity.twist.linear.y = objects[i].getAbsoluteVelocity().getY();
    oa.objects[i].velocity.twist.linear.x = objects[i].getAbsoluteVelocity().getX();
    oa.objects[i].velocity.twist.linear.y = objects[i].getAbsoluteVelocity().getY();
    oa.objects[i].velocity.covariance[0] = objects[i].getAbsoluteVelocitySigma().getX();
    oa.objects[i].velocity.covariance[7] = objects[i].getAbsoluteVelocitySigma().getX();

    oa.objects[i].bounding_box_center.position.x = objects[i].getBoundingBoxCenter().getX();
    oa.objects[i].bounding_box_center.position.y = objects[i].getBoundingBoxCenter().getY();
    oa.objects[i].bounding_box_size.x = objects[i].getBoundingBox().getX();
    oa.objects[i].bounding_box_size.y = objects[i].getBoundingBox().getY();

    oa.objects[i].object_box_center.pose.position.x = objects[i].getCenterPoint().getX();
    oa.objects[i].object_box_center.pose.position.y = objects[i].getCenterPoint().getY();
    oa.objects[i].object_box_center.pose.orientation = tf::createQuaternionMsgFromYaw(objects[i].getCourseAngle());
    oa.objects[i].object_box_center.covariance[0] = objects[i].getCenterPointSigma().getX();
    oa.objects[i].object_box_center.covariance[7] = objects[i].getCenterPointSigma().getY();
    oa.objects[i].object_box_center.covariance[35] = objects[i].getCourseAngleSigma();
    oa.objects[i].object_box_size.x = objects[i].getObjectBox().getX();
    oa.objects[i].object_box_size.y = objects[i].getObjectBox().getY();

    datatypes::Polygon2D contour = objects[i].getContourPoints();
    oa.objects[i].contour_points.resize(contour.size());
    for (int j = 0; j < contour.size(); j++)
    {
      oa.objects[i].contour_points[j].x = contour[j].getX();
      oa.objects[i].contour_points[j].y = contour[j].getY();
    }

    //std::cout << objects[i].toString() << std::endl;
  }

  object_pub_.publish(oa);
}


void SickLDMRS::update_config(SickLDMRSDriverConfig &new_config, uint32_t level)
{
  validate_config(new_config);
  config_ = new_config;

  std::cout << "start_angle:    " << config_.start_angle << std::endl;
  std::cout << "end_angle:      " << config_.end_angle << std::endl;
  std::cout << "frame_id:       " << config_.frame_id << std::endl;
  std::cout << "scan_frequency: " << config_.scan_frequency << std::endl;

  devices::LDMRS* ldmrs;
  ldmrs = dynamic_cast<devices::LDMRS*>(manager_->getFirstDeviceByType(Sourcetype_LDMRS));
  if (ldmrs == NULL)
  {
    ROS_WARN("update_config: no connection to LDMRS!");
    return;
  }

  // TODO: if (new_config.start_angle < config_.end_angle): first update end angle,
  // then start angle to ensure that always start_angle > end_angle; see comments
  // in LuxBase::cmd_setScanAngles().
  ldmrs->setScanAngles(new_config.start_angle, new_config.end_angle);

  switch (config_.scan_frequency)
  {
  case SickLDMRSDriver_ScanFreq1250:
    expected_frequency_ = 12.5d;
    break;
  case SickLDMRSDriver_ScanFreq2500:
    expected_frequency_ = 25.0d;
    break;
  case SickLDMRSDriver_ScanFreq5000:
    expected_frequency_ = 50.0d;
    break;
  default:
    ROS_ERROR("Unknown scan frequency: %i", config_.scan_frequency);
    break;
  }

  ldmrs->setScanFrequency(expected_frequency_);
  ldmrs->setSyncAngleOffset(config_.sync_angle_offset);
  ldmrs->setParameter(devices::ParaContourPointDensity, config_.contour_point_density);
  ldmrs->setParameter(devices::ParaMinimumObjectAge, config_.min_object_age);
  ldmrs->setParameter(devices::ParaMaximumPredictionAge, config_.max_prediction_age);
  ldmrs->setParameter(devices::ParaAngularResolutionType, config_.angular_resolution_type);
  ldmrs->setParameter(devices::ParaRangeReduction, config_.layer_range_reduction);
  ldmrs->setParameter(devices::ParaIgnoreNearRange, config_.ignore_near_range ? 1 : 0);
  ldmrs->setParameter(devices::ParaSensitivityControl, config_.sensitivity_control ? 1 : 0);
}

} /* namespace sick_ldmrs_driver */


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs800001s01");
  boost::shared_ptr<diagnostic_updater::Updater> diagnostics
    = boost::make_shared<diagnostic_updater::Updater>();

  // The MRS-App connects to an MRS, reads its configuration and receives all incoming data.
  // First, create the manager object. The manager handles devices, collects
  // device data and forwards it to the application(s).
  ROS_INFO("Creating the manager.");
  Manager manager;

  // Add the application. As the devices may send configuration data only once
  // at startup, the applications must be present before the devices are
  // started.
  Sourcetype type;
  std::string name;
  UINT16 id;
  bool result = false;

  ROS_INFO("Adding the application SickLDMRS.");
  name = "Sick LDMRS ROS Driver App";
  id = 1356;

  sick_ldmrs_driver::SickLDMRS app(&manager, diagnostics);
  app.setApplicationName(name);

  result = manager.addApplication(&app, id);
  if (result == false)
  {
    ROS_ERROR("Failed to add application %s, aborting!", name.c_str());
    return EXIT_FAILURE;
  }
  ROS_INFO("Application is running.");

  //
  // Add and run the sensor
  //
  // The MRS device could be configured like this:
  // m_weWantScanData:          true
  // m_weWantObjectData:        true
  // m_weWantFieldData:         false
  // m_weWantScanDataFromSopas: false
  ROS_INFO("Adding the LDMRS device.");
  devices::LDMRS* ldmrs = new devices::LDMRS(&manager);
  ldmrs->setWeWantObjectData(true);
  name = "LDMRS-1";
  id = 1;
  result = manager.addAndRunDevice(ldmrs, name, id);
  if (result == false)
  {
    ROS_ERROR("Failed to add device %s, aborting!", name.c_str());
    return EXIT_FAILURE;
  }

  std::string serial_number = ldmrs->getSerialNumber();
  diagnostics->setHardwareID(serial_number);

  ROS_INFO("%s is initialized.", ros::this_node::getName().c_str());

  ros::Rate r(10.0);
  while (ros::ok())
  {
    ros::spinOnce();
    diagnostics->update();
    r.sleep();
  }

  return EXIT_SUCCESS;
}
