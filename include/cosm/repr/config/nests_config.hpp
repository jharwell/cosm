/**
 * \file nests_config.hpp
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

#include "cosm/cosm.hpp"
#include "cosm/repr/config/nest_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct nests_config
 * \ingroup repr config
 *
 * \brief Configuration for the \ref crepr::nest(s) within the arena.
 */
struct nests_config final : public rconfig::base_config {
  std::vector<nest_config> nests{};
};

} /* namespace cosm::repr::config */

