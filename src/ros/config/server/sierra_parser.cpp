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
    : ER_CLIENT_INIT("cosm.ros.config.server.sierra_parser") {
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

  ::ros::NodeHandle nh;
  int tmp;

  std::string param;
  ER_ASSERT(nh.searchParam("sierra/experiment/length", param),
            "Couldn't resolve parameter 'sierra/experiment/length'");
  nh.getParam(param, tmp);
  m_config->experiment.length = rtypes::timestep(tmp);

  ER_ASSERT(nh.searchParam("sierra/experiment/param_file", param),
            "Couldn't resolve parameter 'sierra/experiment/param_file'");
  nh.getParam(param, m_config->experiment.param_file);

  ER_ASSERT(nh.searchParam("sierra/experiment/ticks_per_sec", param),
            "Couldn't resolve parameter 'sierra/experiment/ticks_per_sec'");
  nh.getParam(param, tmp);
  m_config->experiment.ticks_per_sec = rtypes::hertz(tmp);

  /* n_robots not present on robot configuration */
  if (nh.searchParam("sierra/experiment/n_robots", param)) {
    tmp = -1;
    nh.getParam(param, tmp);
    ER_DEBUG("Found n_robots: '%s'=%d", param.c_str(), tmp);
    m_config->experiment.n_robots = tmp;
  }

  /* barrier_start not required */
  if (nh.searchParam("sierra/experiment/barrier_start", param)) {
    bool start;
    nh.getParam(param, start);
    ER_DEBUG("Found barrier_start: '%s'=%d", param.c_str(), start);
    m_config->experiment.barrier_start = start;
  }
  ER_INFO("length=%zu,param_file=%s,ticks_per_sec=%d,n_robots=%zu,barrier_start=%"
          "d",
          m_config->experiment.length.v(),
          m_config->experiment.param_file.c_str(),
          m_config->experiment.ticks_per_sec.v(),
          m_config->experiment.n_robots,
          m_config->experiment.barrier_start);
} /* parse() */

bool sierra_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->experiment.length > 0U,
             "Experiment length cannot be negative");
    ER_CHECK(!m_config->experiment.param_file.empty(),
             "Experiment param file cannot be empty");
    ER_CHECK(m_config->experiment.n_robots > 0U, "# robots must be > 0");
    ER_CHECK(m_config->experiment.ticks_per_sec > 0U,
             "Controller rate cannot be negative");
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(server, config, ros, cosm);
