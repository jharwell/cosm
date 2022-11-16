/**
 * \file ucb1_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
 * \struct ucb1_config
 * \ingroup ta config
 *
 * \brief Configuration for the UCB1 task allocation algorithm, as described in
 * \todo paper ref.
 */
struct ucb1_config final : public rcppsw::config::base_config {
  double gamma{-1};
};

NS_END(config, ta, cosm);

