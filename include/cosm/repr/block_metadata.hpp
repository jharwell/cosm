/**
 * \file block_metadata.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics.hpp"
#include "cosm/repr/block_type.hpp"
#include "cosm/repr/colored_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_metadata
 * \ingroup repr
 *
 * \brief Metadata for blocks which robots can carry and manipulate within the
 * arena, including implementation of \ref metrics::blocks::transport_metrics
 * and what color the block is. This does NOT include the ID of the block, which
 * is part of its core definition.
 */
class block_metadata final : public cfmetrics::block_transportee_metrics,
                             public colored_entity {
 public:
  /**
   * \param color The color of the block.
   *
   * Using this constructor, blocks are assigned the next available id, starting
   * from 0.
   */
  explicit block_metadata(const rutils::color& color,
                          const crepr::block_type& type)
      : colored_entity(color), m_type(type) {}

  ~block_metadata(void) override = default;

  /* transport metrics */
  void reset_metrics(void) override final {
    m_transporters = 0;
    m_first_pickup_time = rtypes::timestep(0);
    m_first_pickup = false;
    m_dist_time = rtypes::timestep(0);
    m_dest_drop_time = rtypes::timestep(0);
  }

  size_t total_transporters(void) const override { return m_transporters; }

  rtypes::timestep total_transport_time(void) const override RCPPSW_PURE {
    return m_dest_drop_time - m_first_pickup_time;
  }

  rtypes::timestep initial_wait_time(void) const override RCPPSW_PURE {
    return m_first_pickup_time - m_dist_time;
  }
  crepr::block_type type(void) const override { return m_type; }

  /**
   * \brief Update a block's state given that it has been picked up by a robot.
   *
   * - Increment the # of carries this block has undergone on its way to its
   *   final destination.
   * - Set its reference to the robot that carries it.
   */
  void robot_pickup_event(const rtypes::type_uuid& robot_id) {
    ++m_transporters;
    m_robot_id = robot_id;
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
   * (e.g. the nest, a structure).
   *
   * \param t The current simulation time.
   */
  void dest_drop_time(const rtypes::timestep& t) { m_dest_drop_time = t; }

  /**
   * \brief Set the time that the block was distributed/dropped in the arena.
   *
   * \param t The current simulation time.
   */
  void distribution_time(const rtypes::timestep& t) { m_dist_time = t; }

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
  void robot_id_reset(void) { m_robot_id = rtypes::constants::kNoUUID; }

  /**
   * \brief Provided to block classes implementing \ref clonable::clone() so
   * that they can correctly clone block metadata/metrics.
   *
   * Note that the robot ID is not copied.
   */
  void metrics_copy(const block_metadata* const other) {
    this->m_transporters = other->m_transporters;
    this->m_first_pickup_time = other->m_first_pickup_time;
    this->m_first_pickup = other->m_first_pickup;
    this->m_dist_time = other->m_dist_time;
    this->m_dest_drop_time = other->m_dest_drop_time;
  }

 private:
  /* clang-format off */
  rtypes::type_uuid m_robot_id{rtypes::constants::kNoUUID};
  size_t            m_transporters{0};
  bool              m_first_pickup{false};
  rtypes::timestep  m_first_pickup_time{0};
  rtypes::timestep  m_dist_time{0};
  rtypes::timestep  m_dest_drop_time{0};
  crepr::block_type m_type;
  /* clang-format on */
};

NS_END(repr, cosm);
