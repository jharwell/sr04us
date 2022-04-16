/**
 * \file driver.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * This file is part of ROSBRIDGE.
 *
 * ROSBRIDGE is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "sr04us/ping.h"

#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(rosbridge, sr04us);

using ping = ::sr04us::ping;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class driver
 * \ingroup rosbridge
 *
 * \brief Driver for the HC-SR04 ultrasonic sensor.
 */
class driver : public rer::client<driver> {
 public:
  driver(void);

  /* Not move/copy constructable/assignable by default */
  driver(const driver&) = delete;
  driver& operator=(const driver&) = delete;
  driver(driver&&) = delete;
  driver& operator=(driver&&) = delete;

  bool report(ping::Request &req, ping::Response &res);

 private:
  void gpio_configure(int trig, int echo);
  float distance_measure(int trig, int echo);


  /* clang-format off */
  /* clang-format on */
};


NS_END(sr04us, rosbridge);
