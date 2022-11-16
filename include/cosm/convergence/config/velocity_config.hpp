/**
 * \file velocity_config.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct velocity_config
 * \ingroup convergence config
 *
 * \brief Configuration for the velocity convergence measure, as described
 * in \todo ref here.
 */
struct velocity_config final : public rconfig::base_config {
  bool enable{false};
};

NS_END(config, convergence, cosm);

