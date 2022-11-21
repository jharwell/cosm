/**
 * \file boid_vector.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "cosm/apf2D/boid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D {

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
using boid_vectorro = std::vector<const boid*>;

} /* namespace cosm::apf2D */
