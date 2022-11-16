/**
 * \file block_embodiment_variant.hpp
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
NS_START(cosm, argos);

struct ramp_block_embodiment;
struct cube_block_embodiment;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using block_embodiment_variant =
    std::variant<std::unique_ptr<ramp_block_embodiment>,
                 std::unique_ptr<cube_block_embodiment> >;

NS_END(argos, cosm);
