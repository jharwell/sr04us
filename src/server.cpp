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
#include <sys/time.h>

#include <ros/ros.h>
#include <wiringPi.h>

#include "sr04us_driver/readings.h"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace rosbridge::sr04us_driver {

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/

float dummy_load1() {
  return 5.12345;
}

float dummy_load2() {
  return 6.12345;
}
void gpio_init(int trig, int echo)  {
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
}
float distance_measure(int trig, int echo)    {
  struct timeval tv1;
  struct timeval tv2;
  long start, stop;
  float dis;
  // Sends ultrasonic signal
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  //Detects ultrasonic signal
  while(!(digitalRead(echo) == 1));
  gettimeofday(&tv1, NULL);
  while(!(digitalRead(echo) == 0));
  gettimeofday(&tv2, NULL);
  //Measure gap of time and calculate distance
  start = tv1.tv_sec * 1000000 + tv1.tv_usec;
  stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;
  dis = (float)(stop - start) / 1000000 * 34000 / 2;

  return dis;
}

bool report(::sr04us_driver::readings::Request &req,
            ::sr04us_driver::readings::Response &res)  {
  auto trig = req.trig;
  auto echo = req.echo;
  // gpio_init(trig, echo)
  // res.reading1 = distance_measure(trig, echo);
  // res.reading2 = distance_measure(trig, echo);
  res.reading1 = dummy_load1();
  res.reading2 = dummy_load2();
  ROS_INFO("request: Trig:%ld, Echo:%ld", (long int)req.trig, (long int)req.echo);
  ROS_INFO("sending back reading1: [%ld]\nreading2:[%ld]", (long int)res.reading1, (long int)res.reading2);
  return true;
}

} /* namespace rosbridge::sr04us_driver */

int main(int argc, char **argv) {
    ros::init(argc, argv, "sr04us_driver");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("sr04us_driver", rosbridge::sr04us_driver::report);
    ROS_INFO("Ready to send readings.");
    ros::spin();

    return 0;
}
