/**
 * \file epsilon_greedy_config.hpp
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

#ifndef INCLUDE_COSM_TA_CONFIG_EPSILON_GREEDY_CONFIG_HPP_
#define INCLUDE_COSM_TA_CONFIG_EPSILON_GREEDY_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct epsilon_greedy_config
 * \ingroup ta config
 *
 * \brief Configuration for the \f$\epsilon\f$-greedy task allocation method, as
 * described in \todo paper ref.
 */
struct epsilon_greedy_config final : public rcppsw::config::base_config {
  double epsilon{-1};
  std::string regret_bound{};
};

NS_END(config, ta, cosm);

#endif /* INCLUDE_COSM_TA_CONFIG_EPSILON_GREEDY_CONFIG_HPP_ */
