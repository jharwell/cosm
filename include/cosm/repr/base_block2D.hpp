/**
 * \file base_block2D.hpp
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

#ifndef INCLUDE_COSM_REPR_BASE_BLOCK2D_HPP_
#define INCLUDE_COSM_REPR_BASE_BLOCK2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/metrics/blocks/transport_metrics.hpp"
#include "cosm/repr/block_type.hpp"
#include "cosm/repr/colored_entity.hpp"
#include "cosm/repr/unicell_movable_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_block2D
 * \ingroup cosm repr
 *
 * \brief Base class for representing blocks (i.e. things that robots carry
 * within the arena) as 2D entities (for simplicity). Blocks have both real
 * (where they actually live in the world) and discretized locations (where they
 * are mapped to within the arena).
 */
class base_block2D : public unicell_movable_entity2D,
                     public colored_entity,
                     public metrics::blocks::transport_metrics,
                     public rpprototype::clonable<base_block2D> {
 public:
  /**
   * \brief Out of sight location blocks are moved to when a robot picks them
   * up, for visualization/rendering purposes.
   */
  static constexpr rmath::vector2u kOutOfSightDLoc = rmath::vector2u(1000, 1000);
  static constexpr rmath::vector2d kOutOfSightRLoc =
      rmath::vector2d(1000.0, 1000.0);

  /**
   * \param dim 2 element vector of the dimensions of the block.
   * \param color The color of the block.
   *
   * Using this constructor, blocks are assigned the next available id, starting
   * from 0.
   */
  base_block2D(const rmath::vector2d& dim, const rutils::color& color)
      : unicell_movable_entity2D(dim, rtypes::constants::kNoUUID),
        colored_entity(color) {}

  /**
   * \param dim 2 element vector of the dimensions of the block.
   * \param color The color of the block.
   * \param id The id of the block.
   */
  base_block2D(const rmath::vector2d& dim,
               const rutils::color& color,
               const rtypes::type_uuid& id)
      : unicell_movable_entity2D(dim, id), colored_entity(color) {}

  ~base_block2D(void) override = default;

  /**
   * \brief Disallow direct object comparisons, because we may want to compare
   * for equality in terms of IDs or object locations, and it is better to
   * require explicit comparisons for BOTH, rather than just one. It also makes
   * it unecessary to have to remember which type the comparison operator==()
   * does for this class.
   */
  bool operator==(const base_block2D& other) const = delete;

  /**
   * \brief Compare two \ref base_block2D objects for equality based on their
   * ID.
   */
  bool idcmp(const base_block2D& other) const {
    return this->id() == other.id();
  }

  /**
   * \brief Compare two \ref base_block2D objects for equality based on their
   * discrete location.
   */
  bool dloccmp(const base_block2D& other) const {
    return this->dloc() == other.dloc();
  }

  /* transport metrics */
  void reset_metrics(void) override final {
    m_transporters = 0;
    m_first_pickup_time = rtypes::timestep(0);
    m_first_pickup = false;
    m_dist_time = rtypes::timestep(0);
    m_dest_drop_time = rtypes::timestep(0);
  }

  uint total_transporters(void) const override { return m_transporters; }

  rtypes::timestep total_transport_time(void) const override RCSW_PURE {
    return m_dest_drop_time - m_first_pickup_time;
  }

  rtypes::timestep initial_wait_time(void) const override RCSW_PURE {
    return m_first_pickup_time - m_dist_time;
  }

  /**
   * \brief Update a block's state given that it has been picked up by a robot.
   *
   * - Increment the # of carries this block has undergone on its way to its
   *   final destination.
   * - Set its reference to the robot that carries it.
   * - Move the block's position out of sight so that it is not discoverable by
   *   other robots.
   */
  void robot_pickup_event(const rtypes::type_uuid& robot_id) {
    ++m_transporters;
    m_robot_id = robot_id;
    move_out_of_sight();
  }

  /**
   * \brief Set the time that the block is picked up for the first time
   * after being distributed in the arena.
   *
   * \param t The current simulation time.
   */
  void first_pickup_time(const rtypes::timestep& t) {
    if (!m_first_pickup) {
      m_first_pickup_time = t;
      m_first_pickup = true;
    }
  }

  /**
   * \brief Set the time that the block is dropped at its destination location
   * (eg the nest, a structure).
   *
   * \param t The current simulation time.
   */
  void dest_drop_time(const rtypes::timestep& t) { m_dest_drop_time = t; }

  /**
   * \brief Set the time that the block was distributed in the arena.
   *
   * \param t The current simulation time.
   */
  void distribution_time(const rtypes::timestep& t) { m_dist_time = t; }

  /**
   * \brief Determine if the block is currently out of sight.
   *
   * This should only happen if the block is being carried by a robot.
   */
  bool is_out_of_sight(void) const {
    return kOutOfSightDLoc == dloc() || kOutOfSightRLoc == rloc();
  }

  /**
   * \return The robot index, or -1 if no robot is currently carrying this
   * block.
   */
  const rtypes::type_uuid& robot_id(void) const { return m_robot_id; }
  void robot_id(const rtypes::type_uuid& id) { m_robot_id = id; }

  /**
   * \brief Reset the the blocks carried/not carried state when it is not
   * carried by a robot anymore, but has not yet made it to its final
   * destination.
   */
  void reset_robot_id(void) { m_robot_id = rtypes::constants::kNoUUID; }

 protected:
  /**
   * \brief Provided to derived classes implementing \ref clonable::clone() so
   * that they can correctly clone block metadata/metrics.
   */
  void copy_metrics(const base_block2D& other) {
    this->m_transporters = other.m_transporters;
    this->m_first_pickup_time = other.m_first_pickup_time;
    this->m_first_pickup = other.m_first_pickup;
    this->m_dist_time = other.m_dist_time;
    this->m_dest_drop_time = other.m_dest_drop_time;
  }

 private:
  /**
   * \brief Change the block's location to something outside the visitable space
   * in the arena when it is being carried by robot.
   */
  void move_out_of_sight(void) {
    rloc(kOutOfSightRLoc);
    dloc(kOutOfSightDLoc);
  }

  /* clang-format off */
  rtypes::type_uuid m_robot_id{rtypes::constants::kNoUUID};
  uint                m_transporters{0};
  bool                m_first_pickup{false};
  rtypes::timestep    m_first_pickup_time{0};
  rtypes::timestep    m_dist_time{0};
  rtypes::timestep    m_dest_drop_time{0};
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_BLOCK2D_HPP_ */
