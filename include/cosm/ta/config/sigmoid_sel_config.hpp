/**
 * \file sigmoid_sel_config.hpp
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

#ifndef INCLUDE_COSM_TA_CONFIG_SIGMOID_SEL_CONFIG_HPP_
#define INCLUDE_COSM_TA_CONFIG_SIGMOID_SEL_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/math/config/sigmoid_config.hpp"
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct sigmoid_sel_config
 * \ingroup config ta
 *
 * \brief Configuration for sigmoid derived probabilities to choose between
 * multiple options using one or more methods.
 *
 */
struct sigmoid_sel_config final : public rcppsw::config::base_config {
  std::string method{};
  rmath::config::sigmoid_config sigmoid{};
};

NS_END(config, ta, cosm);

#endif /* INCLUDE_COSM_TA_CONFIG_SIGMOID_SEL_CONFIG_HPP_ */
