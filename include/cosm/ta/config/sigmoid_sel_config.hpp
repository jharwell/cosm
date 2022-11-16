/**
 * \file sigmoid_sel_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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

