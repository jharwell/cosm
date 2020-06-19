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

#ifndef INCLUDE_COSM_REPR_BASE_BLOCK3D_HPP_
#define INCLUDE_COSM_REPR_BASE_BLOCK3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/repr/block_metadata.hpp"
#include "cosm/repr/unicell_movable_entity3D.hpp"
#include "cosm/repr/operations/block_pickup_owner.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_block3D
 * \ingroup cosm repr
 *
 * \brief Base class for representing blocks (i.e. things that robots carry
 * within the arena). Blocks have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena).
 */
class base_block3D : public crepr::unicell_movable_entity3D,
                     public rpprototype::clonable<base_block3D> {
 public:
  /**
   * \param dim 2 element vector of the dimensions of the block.
   * \param color The color of the block.
   * \param type The type of the block.
   *
   * Using this constructor, blocks are assigned the next available id, starting
   * from 0.
   */
  base_block3D(const rmath::vector3d& dim,
               const rutils::color& color,
               const crepr::block_type& type)
      : unicell_movable_entity3D(dim, rtypes::constants::kNoUUID),
        m_md(color, type) {}

  /**
   * \param dim 3 element vector of the dimensions of the block.
   * \param color The color of the block.
   * \param type The type of the block.
   * \param id The id of the block.
   */
  base_block3D(const rmath::vector3d& dim,
               const rutils::color& color,
               const crepr::block_type& type,
               const rtypes::type_uuid& id)
      : unicell_movable_entity3D(dim, id), m_md(color, type) {}

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
  bool idcmp(const base_block3D& other) const {
    return this->id() == other.id();
  }

  /**
   * \brief Compare two \ref base_block3D objects for equality based on their
   * discrete location.
   */
  bool dloccmp(const base_block3D& other) const {
    return this->dpos3D() == other.dpos3D();
  }

  const block_metadata* md(void) const { return &m_md; }
  block_metadata* md(void) { return &m_md; }

  /**
   * \brief Update block state given that it has been picked up.
   *
   * This function does NOT move the block out of sight.
   */
  void update_on_pickup(const rtypes::type_uuid& robot_id,
                        const rtypes::timestep& t,
                        const crops::block_pickup_owner& owner) {
    switch (owner) {
      case crops::block_pickup_owner::ekARENA_MAP:
        move_out_of_sight();
        md()->robot_id(robot_id); /* needed to mark block as "in-use" */
        break;
      case crops::block_pickup_owner::ekROBOT:
        m_md.robot_pickup_event(robot_id);
        m_md.first_pickup_time(t);
        break;
      default:
        break;
    } /* switch() */
  }

  /**
   * \brief Determine if the block is currently out of sight.
   *
   * This should only happen if the block is being carried by a robot.
   */
  bool is_out_of_sight(void) const {
    return kOutOfSight.dpos == unicell_movable_entity3D::dpos3D() ||
           kOutOfSight.rpos == unicell_movable_entity3D::rpos3D();
  }
  /**
   * \brief Change the block's location to something outside the visitable space
   * in the arena when it is being carried by robot.
   */
  void move_out_of_sight(void) {
    unicell_movable_entity3D::rpos3D(kOutOfSight.rpos);
    unicell_movable_entity3D::dpos3D(kOutOfSight.dpos);
  }

 private:
  /**
   * \brief Out of sight location blocks are moved to when a robot picks them
   * up, for visualization/rendering purposes.
   */
  struct out_of_sight3D {
    rmath::vector3d rpos{1000.0, 1000.0, 0.0};
    rmath::vector3z dpos{1000, 1000, 0};
  };

  static const out_of_sight3D kOutOfSight;


  /* clang-format off */
  block_metadata m_md;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_BLOCK3D_HPP_ */
