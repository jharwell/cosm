/**
 * \file angular_order_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
namespace cosm::convergence::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct angular_order_config
 * \ingroup convergence config
 *
 * \brief Configuration for the angular order convergence measure, as described
 * in \todo ref here.
 */
struct angular_order_config final : public rconfig::base_config {
  bool enable{false};
};

} /* namespace cosm::convergence::config */

