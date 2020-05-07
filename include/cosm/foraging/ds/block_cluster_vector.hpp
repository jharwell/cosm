/**
 * \file block_cluster_vector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_DS_BLOCK_CLUSTER_VECTOR_HPP_
#define INCLUDE_COSM_FORAGING_DS_BLOCK_CLUSTER_VECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

namespace cosm::foraging::repr {
class block_cluster;
} // namespace repr

NS_START(cosm, foraging, ds);

using block3D_cluster_vector_type = const cfrepr::block_cluster*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
using block3D_cluster_vector = std::vector<block3D_cluster_vector_type>;

NS_END(ds, foraging, cosm);

#endif /* INCLUDE_COSM_DS_BLOCK_CLUSTER_VECTOR_HPP_ */
