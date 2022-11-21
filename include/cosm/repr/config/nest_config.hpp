/**
 * \file nest_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct nest_config
 * \ingroup repr config
 *
 * \brief Configuration for a single \ref nest within the arena.
 */
struct nest_config final : public rconfig::base_config {
  rmath::vector2d center{};
  rmath::vector2d dims{};
};

} /* namespace cosm::repr::config */

