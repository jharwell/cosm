/**
 * \file output_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_METRICS_CONFIG_OUTPUT_CONFIG_HPP_
#define INCLUDE_COSM_METRICS_CONFIG_OUTPUT_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "cosm/metrics/config/metrics_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct output_config
 * \ingroup cosm metrics config
 *
 * \brief Configuration for metrics logging.
 */
struct output_config final : public rconfig::base_config {
  std::string            output_root{};
  std::string            output_dir{};
  metrics_config metrics {};
};

NS_END(config, metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_CONFIG_OUTPUT_CONFIG_HPP_ */
