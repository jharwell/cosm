/**
 * \file sierra_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ros/config/server/sierra_parser.hpp"

#include <ros/ros.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, config, server);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
sierra_parser::sierra_parser(void)
    : ER_CLIENT_INIT("cosm.ros.config.server.sierra_parser"),
      mc_names({
          "/sierra/experiment/length",
          "/sierra/experiment/param_file",
          "/sierra/experiment/n_robots",
          "/sierra/experiment/ticks_per_sec"
        }) {
  /*
   * This parser can be the FIRST thing to run which uses logging, so make sure
   * it is setup.
   */
  ER_ENV_VERIFY();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sierra_parser::parse(void) {
  /* Always parsed */
  m_config = std::make_unique<config_type>();

  ::ros::NodeHandle nh("~");
  int tmp;

  check_for_param(mc_names[0]);
  nh.getParam(mc_names[0], tmp);
  m_config->experiment.length = rtypes::timestep(tmp);

  check_for_param(mc_names[1]);
  nh.getParam(mc_names[1], m_config->experiment.param_file);

  /* n_robots not present on robot configuration */
  nh.getParam(mc_names[2], tmp);
  m_config->experiment.n_robots = tmp;

  check_for_param(mc_names[3]);
  nh.getParam(mc_names[3], tmp);
  m_config->experiment.ticks_per_sec = rtypes::hertz(tmp);
} /* parse() */

bool sierra_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->experiment.length > 0U,
             "Experiment length cannot be negative");
    ER_CHECK(!m_config->experiment.param_file.empty(),
             "Experiment param file cannot be empty");
    ER_CHECK(m_config->experiment.n_robots > 0U,
             "# robots must be > 0");
    ER_CHECK(m_config->experiment.ticks_per_sec > 0U,
             "Controller rate cannot be negative");
  }
  return true;

error:
  return false;
} /* validate() */

void sierra_parser::check_for_param(const std::string& name) const {
  ::ros::NodeHandle nh("~");
  ER_ASSERT(nh.hasParam(name), "Parameter '%s' does not exist", name.c_str());
} /* check_for_param() */

NS_END(server, config, ros, cosm);
