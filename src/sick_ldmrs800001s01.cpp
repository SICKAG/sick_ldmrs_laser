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

#include "sick_ldmrs_driver/sick_ldmrs800001s01.hpp"

#include <sick_ldmrs/datatypes/EvalCaseResults.hpp>
#include <sick_ldmrs/datatypes/EvalCases.hpp>
#include <sick_ldmrs/datatypes/Fields.hpp>
#include <sick_ldmrs/datatypes/Measurement.hpp>
#include <sick_ldmrs/datatypes/Msg.hpp>
#include <sick_ldmrs/datatypes/Object.hpp>
#include <sick_ldmrs/datatypes/Scan.hpp>

#include <sick_ldmrs/tools/errorhandler.hpp>
#include <sick_ldmrs/tools/toolbox.hpp>


sick_ldmrs_driver::SickLDMRS::SickLDMRS(Manager *manager)
  : application::BasicApplication()
{
  // TODO MG
}

void sick_ldmrs_driver::SickLDMRS::setData(BasicData &data)
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
      // Print the scan start timestamp (NTP time)
      Scan* scan = dynamic_cast<Scan*>(&data);
      const ScannerInfo* info = scan->getScannerInfoByDeviceId(1);

      if (info != NULL)
      {
        const Time& time = info->getStartTimestamp();
        ROS_INFO("LdmrsApp::setData(): Scan start time: %s", time.toString().c_str());
      }

      // TODO MG: convert to PointCloud, publish
    }
    break;
  case Datatype_Objects:
    datatypeStr = "Objects (" + ::toString(((ObjectList&)data).size()) + " objects)";
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_ldmrs800001s01");

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

  application::BasicApplication* app;
  app = new sick_ldmrs_driver::SickLDMRS(&manager);
  app->setApplicationName(name);

  result = manager.addApplication(app, id);
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
  type = Sourcetype_LDMRS;
  name = "LDMRS-1";
  id = 1;
  result = manager.addAndRunDevice(type, name, id);
  if (result == false)
  {
    ROS_ERROR("Failed to add device %s, aborting!", name.c_str());
    return EXIT_FAILURE;
  }

  // This loop never ends
  while (1)
  {
    // Sleep 100 ms
    usleep(100000);
  }

  return EXIT_SUCCESS;
}
