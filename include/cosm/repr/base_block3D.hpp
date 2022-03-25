/**
 * \file base_block.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/repr/block_metadata.hpp"
#include "cosm/repr/operations/block_pickup_owner.hpp"
#include "cosm/repr/unicell_movable_entity3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_block3D
 * \ingroup repr
 *
 * \brief Base class for representing blocks (i.e. things that robots carry
 * within the arena). Blocks have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena).
 */
class base_block3D : public crepr::unicell_movable_entity3D,
                     public rpprototype::clonable<base_block3D> {
 public:
  /**
   * \param dim 3 element vector of the dimensions of the block.
   * \param color The color of the block.
   * \param type The type of the block.
   * \param id The id of the block.
   */
  base_block3D(const rtypes::type_uuid& id,
               const rmath::vector3d& dim,
               const rtypes::discretize_ratio& arena_res,
               const rutils::color& color,
               const crepr::block_type& type)
      : unicell_movable_entity3D(id, dim, arena_res), m_md(color, type) {}

  ~base_block3D(void) override = default;

  /**
   * \brief Disallow direct object comparisons, because we may want to compare
   * for equality in terms of IDs or object locations, and it is better to
   * require explicit comparisons for BOTH, rather than just one. It also makes
   * it unecessary to have to remember which type the comparison operator==()
   * does for this class.
   */
  bool operator==(const base_block3D& other) const = delete;

  /**
   * \brief Compare two \ref base_block3D objects for equality based on their
   * ID.
   */
  bool idcmp(const base_block3D& other) const { return this->id() == other.id(); }

  /**
   * \brief Compare two \ref base_block3D objects for equality based on their
   * discrete location by comparing their anchors.
   */
  bool dloccmp(const base_block3D& other) const {
    return this->danchor3D() == other.danchor3D();
  }

  const block_metadata* md(void) const { return &m_md; }
  block_metadata* md(void) { return &m_md; }

  bool is_carried_by_robot(void) const {
    return rtypes::constants::kNoUUID != m_md.robot_id();
  }

  /**
   * \brief Update block state given that it has been picked up.
   */
  virtual void update_on_pickup(const rtypes::type_uuid& robot_id,
                                const rtypes::timestep& t,
                                const crops::block_pickup_owner& owner) = 0;

 protected:
  void clone_impl(base_block3D* const other) const {
    /* copy core definition features */
    other->ranchor3D(this->ranchor3D());
    other->danchor3D(this->danchor3D());

    /* copy metadata */
    other->md()->robot_id_reset();
    other->md()->metrics_copy(this->md());
  }

 private:
  /* clang-format off */
  block_metadata m_md;
  /* clang-format on */
};

NS_END(repr, cosm);

