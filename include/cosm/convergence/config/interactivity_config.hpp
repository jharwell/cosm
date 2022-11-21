/**
 * \file interactivity_config.hpp
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
 * \struct interactivity_config
 * \ingroup convergence config
 *
 * \brief Configuration for the interactivity convergence measure, as described
 * in \todo paper ref.
 */
struct interactivity_config final : public rconfig::base_config {
  bool enable{false};
};

} /* namespace cosm::convergence::config */

