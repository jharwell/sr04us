/**
 * \file test_client.cpp
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

#include "sr04us/driver.hpp"
#include "sr04us/ping_service.hpp"

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "sr04us_test_client");
  if (argc != 3)  {
    ROS_INFO("usage: sr04us_test_client [trigger_pin] [echo_pin]");
    return 1;
  }

  ros::NodeHandle n;
  auto client = n.serviceClient<sr04us::PingService>(rosbridge::sr04us::kServiceName);
  sr04us::PingService srv;
  srv.request.trig = atoll(argv[1]);
  srv.request.echo = atoll(argv[2]);

  ros::Rate rate(1); /* 1 Hz */

  while (ros::ok()) {
    if (client.call(srv))   {
      ROS_INFO("Reading1: %lf Reading2: %lf",
               (float)srv.response.readings[0].value,
               (float)srv.response.readings[1].value);
    } else {
      ROS_ERROR("Failed to call service %s",
                rosbridge::sr04us::kServiceName);
    }
    ros::spinOnce();
    rate.sleep();
  } /* while() */

  return 0;
}
