/**
 * \file embodied_block_variant.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <variant>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

namespace cosm::argos {

class embodied_ramp_block;
class embodied_cube_block;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using embodied_block_varianto =
    std::variant<std::unique_ptr<embodied_ramp_block>,
                 std::unique_ptr<embodied_cube_block> >;

using embodied_block_variantno =
    std::variant<embodied_ramp_block*, embodied_cube_block*>;

} /* namespace cosm::argos */
