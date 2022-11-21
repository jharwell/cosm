/**
 * \file block_cluster_params.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::foraging::repr {

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

} /* namespace cosm::foraging::repr */

