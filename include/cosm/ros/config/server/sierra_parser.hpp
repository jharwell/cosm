/**
 * \file sierra_parser.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>
#include <array>

#include "rcppsw/config/server/server_config_parser.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/ros/config/sierra_config.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, config, server);

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
  void check_for_param(const std::string& name) const;
  /* clang-format off */
  const std::array<std::string, 4> mc_names;
  std::unique_ptr<config_type>     m_config{nullptr};
  /* clang-format on */
};

NS_END(server, config, ros, cosm);
