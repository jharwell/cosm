/**
 * \file entity_dimensionality.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitionsn
 ******************************************************************************/
/**
 * \brief The different dimensionalitys of entities available in simulation.
 */
enum class entity_dimensionality {
  ek2D,
  ek3D,
};

NS_END(repr, cosm);
