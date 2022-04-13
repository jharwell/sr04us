/**
 * \file service.cpp
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
#include <sys/time.h>

#include <ros/ros.h>

#include "sr04us_driver/sr04us_driver.hpp"

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
int main(int argc, char **argv) {
    ros::init(argc, argv, "sr04us_service");
    ros::NodeHandle n;

    auto service = n.advertiseService(rosbridge::sr04us_driver::kServiceName,
                                      rosbridge::sr04us_driver::report);
    ROS_INFO("Ready to send readings.");

    ros::spin();
    return 0;
}
