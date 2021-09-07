/**
 * \file spatial_entity.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SPATIAL_ENTITY_HPP_
#define INCLUDE_COSM_SPATIAL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/base_entity.hpp"
#include "cosm/repr/entity_dimensionality.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class spatial_entity : public base_entity {
 public:
  spatial_entity(void) : base_entity{ rtypes::constants::kNoUUID } {}
  explicit spatial_entity(const rtypes::type_uuid& id) : base_entity(id) {}

  ~spatial_entity(void) override = default;

  /**
   * \brief Return whether the entity is 2D or 3D.
   */
  virtual entity_dimensionality dimensionality(void) const = 0;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_SPATIAL_ENTITY_HPP_ */
