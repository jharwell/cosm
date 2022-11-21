/**
 * \file sierra_parser.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/config/server/server_config_parser.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/ros/config/sierra_config.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ros::config::server {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sierra_parser
 * \ingroup ros config server
 *
 * \brief Parses server parameters set by SIERRA for use by COSM-derived
 * projects from the ROS parameter server.
 */
class sierra_parser : public rer::client<sierra_parser>,
                      public rconfig::server::server_config_parser {
 public:
  using config_type = sierra_config;

  sierra_parser(void);

  void parse(void) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_COLD;

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */

  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::ros::config::server */
