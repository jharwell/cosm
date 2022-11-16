/**
 * \file sierra_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
  ER_ASSERT(nh.getParam(param, tmp), "%s not found?", param.c_str());
  m_config->experiment.length = rtypes::timestep(tmp);

  ER_ASSERT(nh.searchParam("sierra/experiment/param_file", param),
            "Couldn't resolve parameter 'sierra/experiment/param_file'");
  ER_ASSERT(nh.getParam(param, m_config->experiment.param_file),
            "%s not found?",
            param.c_str());

  ER_ASSERT(nh.searchParam("sierra/experiment/ticks_per_sec", param),
            "Couldn't resolve parameter 'sierra/experiment/ticks_per_sec'");
  ER_ASSERT(nh.getParam(param, tmp), "%s not found?", param.c_str());
  m_config->experiment.ticks_per_sec = rtypes::hertz(tmp);

  /* n_robots not present on robot configuration */
  if (nh.searchParam("sierra/experiment/n_robots", param)) {
    tmp = -1;
    if (nh.getParam(param, tmp)) {
      m_config->experiment.n_robots = tmp;
    }
  }

  /* barrier_start not required */
  if (nh.searchParam("sierra/experiment/barrier_start", param)) {
    bool start = false;
    ER_ASSERT(nh.getParam(param, start), "%s not found?", param.c_str());
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
