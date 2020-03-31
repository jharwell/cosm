/**
 * \file aggregate_oracle_config.hpp
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

#ifndef INCLUDE_COSM_ORACLE_CONFIG_AGGREGATE_ORACLE_CONFIG_HPP_
#define INCLUDE_COSM_ORACLE_CONFIG_AGGREGATE_ORACLE_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"
#include "cosm/oracle/config/tasking_oracle_config.hpp"
#include "cosm/oracle/config/entities_oracle_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct aggregate_oracle_config
 * \ingroup oracle config
 *
 * \brief Parameters for the various oracles that can be employed during
 * simulation.
 */
struct aggregate_oracle_config final : public rconfig::base_config {
  struct coconfig::tasking_oracle_config tasking{};
  struct coconfig::entities_oracle_config entities{};
};

NS_END(config, oracle, cosm);

#endif /* INCLUDE_COSM_ORACLE_CONFIG_AGGREGATE_ORACLE_CONFIG_HPP_ */
