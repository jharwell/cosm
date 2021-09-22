/**
 * \file block_cluster_params.hpp
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

#ifndef INCLUDE_COSM_FORAGING_REPR_BLOCK_CLUSTER_PARAMS_HPP_
#define INCLUDE_COSM_FORAGING_REPR_BLOCK_CLUSTER_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct block_cluster_params
 * \ingroup foraging repr
 *
 * \brief Parameters for \ref block_cluster.
 */

struct block_cluster_params {
  rtypes::type_uuid        id;
  size_t                   capacity;
  rtypes::discretize_ratio resolution;
  cads::arena_grid::view   view;
};

NS_END(repr, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_REPR_BLOCK_CLUSTER_PARAMS_HPP_ */
