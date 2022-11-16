/**
 * \file grid2D_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct grid2D_config
 * \ingroup config ds
 *
 * \brief Configuration for the \ref arena_grid used to represent the arena by
 * both loop functions and robots.
 */
struct grid2D_config final : public rconfig::base_config {
  rtypes::discretize_ratio resolution{0.0};
  rmath::vector2d dims{};
};

NS_END(config, ds, cosm);

