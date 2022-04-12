/**
 * \file client.cpp
 *
 * \copyright 2022 Shizhi Xu, All rights reserved.
 *
 * This file is part of ROSBRIDGE.
 *
 * ROSBRIDGE is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ROSBRIDGE is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ROSBRIDGE.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <cstdlib>
#include <ros/ros.h>

#include "sr04us_driver/readings.h"

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/

int main(int argc, char **argv) {
  ros::init(argc, argv, "sr04us_driver_client");
  if (argc != 3)  {
    ROS_INFO("usage: sr04us_driver_driver Status");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<sr04us_driver::readings>("sr04us_driver");
  sr04us_driver::readings srv;
  srv.request.trig = atoll(argv[1]);
  srv.request.echo = atoll(argv[2]);
  if (client.call(srv))   {
    ROS_INFO("Reading1: %lf\nReading2: %lf", (float)srv.response.reading1, (float)srv.response.reading2);
  }
  else    {
    ROS_ERROR("Failed to call service sr04us_driver");
    return 1;
  }
  return 0;
}
