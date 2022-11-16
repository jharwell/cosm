/**
 * \file entity_vector.hpp
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

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class unicell_entity2D;
class unicell_entity3D;
class base_entity;
class spatial_entity2D;
} // namespace cosm::repr

NS_START(cosm, ds);

using entity2D_vector_type = crepr::unicell_entity2D*;
using const_entity2D_vector_type = const crepr::unicell_entity2D*;

using entity3D_vector_type = crepr::unicell_entity3D*;
using const_entity3D_vector_type = const crepr::unicell_entity3D*;

using entity_vector_type = crepr::base_entity*;
using const_entity_vector_type = const crepr::base_entity*;

/*
 * We use spatial_entity2D rather than the base class, because all
 * spatial_entity3D derived classes are also spatial_entity2D instances.
 */
using spatial_entity_vector_type = crepr::spatial_entity2D*;
using const_spatial_entity_vector_type = const crepr::spatial_entity2D*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
using entity2D_vector = std::vector<entity2D_vector_type>;
using const_entity2D_vector = std::vector<const_entity2D_vector_type>;

using entity3D_vector = std::vector<entity3D_vector_type>;
using const_entity3D_vector = std::vector<const_entity3D_vector_type>;

using entity_vector = std::vector<entity_vector_type>;
using const_entity_vector = std::vector<const_entity_vector_type>;

using spatial_entity_vector = std::vector<spatial_entity_vector_type>;
using const_spatial_entity_vector = std::vector<const_spatial_entity_vector_type>;

NS_END(ds, cosm);
