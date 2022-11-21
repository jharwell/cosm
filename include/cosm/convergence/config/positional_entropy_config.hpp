/**
 * \file positional_entropy_config.hpp
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
#include "rcppsw/math/range.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence::config {

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

} /* namespace cosm::convergence::config */

