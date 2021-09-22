/**
 * \file positional_entropy_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONVERGENCE_CONFIG_POSITIONAL_ENTROPY_CONFIG_HPP_
#define INCLUDE_COSM_CONVERGENCE_CONFIG_POSITIONAL_ENTROPY_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/range.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct positional_entropy_config
 * \ingroup convergence config
 *
 * \brief Configuration for the positional entropy convergence measure, as
 * described in \todo ref here.
 */
struct positional_entropy_config final : public rconfig::base_config {
  positional_entropy_config(void) noexcept = default;

  bool enable{false};
  rmath::ranged horizon{-1, 0};
  double horizon_delta{-1};
};

NS_END(config, convergence, cosm);

#endif /* INCLUDE_COSM_CONVERGENCE_CONFIG_POSITIONAL_ENTROPY_CONFIG_HPP_ */
