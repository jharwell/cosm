/**
 * \file entity_dimensionality.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_REPR_ENTITY_DIMENSIONALITY_HPP_
#define INCLUDE_COSM_REPR_ENTITY_DIMENSIONALITY_HPP_

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

#endif /* INCLUDE_COSM_REPR_ENTITY_DIMENSIONALITY_HPP_ */
