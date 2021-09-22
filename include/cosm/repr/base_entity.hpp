/**
 * \file base_entity.hpp
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

#ifndef INCLUDE_COSM_REPR_BASE_ENTITY_HPP_
#define INCLUDE_COSM_REPR_BASE_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/entity_dimensionality.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_entity
 * \ingroup cosm repr
 *
 * \brief A base class from which all entities that can be represented in the
 * arena derive.
 */
class base_entity {
 public:
  base_entity(void) : base_entity{ rtypes::constants::kNoUUID } {}
  explicit base_entity(const rtypes::type_uuid& id) : m_id(id) {}

  base_entity(const base_entity&) = default;
  base_entity& operator=(const base_entity&) = default;

  virtual ~base_entity(void) = default;

  /**
   * \brief Set the ID of the object.
   */
  void id(const rtypes::type_uuid& id) { m_id = id; }

  /**
   * \brief Get the ID of the object.
   */
  const rtypes::type_uuid& id(void) const { return m_id; }

  /**
   * \brief Set if the ID of the entity should be visualized or not if
   * visualizations are enabled.
   */
  void vis_id(bool b) { m_vis_id = b; }

  /**
   * \brief Get the entity ID visualization status.
   */
  bool vis_id(void) const { return m_vis_id; }

 private:
  /* clang-format off */
  bool              m_vis_id{false};
  rtypes::type_uuid m_id{rtypes::constants::kNoUUID};
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_ENTITY_HPP_ */
