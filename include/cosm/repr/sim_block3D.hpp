/**
 * \file sim_block.hpp
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

#ifndef INCLUDE_COSM_REPR_SIM_BLOCK3D_HPP_
#define INCLUDE_COSM_REPR_SIM_BLOCK3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sim_block3D
 * \ingroup cosm repr
 *
 * \brief Base class for representing blocks (i.e. things that robots carry
 * within the arena) in a simulated environment. Blocks have both real (where
 * they actually live in the world) and discretized locations (where they are
 * mapped to within the arena).
 */
class sim_block3D : public rer::client<sim_block3D>,
                    public crepr::base_block3D {
 public:
  sim_block3D(const rtypes::type_uuid& id,
               const rmath::vector3d& dim,
               const rtypes::discretize_ratio& arena_res,
               const rutils::color& color,
               const crepr::block_type& type)
      : ER_CLIENT_INIT("cosm.repr.sim_block3D"),
        base_block3D(id, dim, arena_res, color, type) {}

  ~sim_block3D(void) override = default;


  /**
   * \brief Determine if the block is currently out of sight.
   *
   * This should only happen if the block is being carried by a robot.
   */
  bool is_out_of_sight(void) const {
    return kOutOfSight.dpos == unicell_movable_entity3D::danchor3D() ||
        kOutOfSight.rpos == unicell_movable_entity3D::ranchor3D();
  }
  /**
   * \brief Change the block's location to something outside the visitable space
   * in the arena when it is being carried by robot.
   */
  void move_out_of_sight(void) {
    unicell_movable_entity3D::ranchor3D(kOutOfSight.rpos);
    unicell_movable_entity3D::danchor3D(kOutOfSight.dpos);
  }

 protected:
  void update_on_pickup(const rtypes::type_uuid& robot_id,
                        const rtypes::timestep& t,
                        const crops::block_pickup_owner& owner) override {
    switch (owner) {
      case crops::block_pickup_owner::ekARENA_MAP:
        move_out_of_sight();
        md()->robot_id(robot_id); /* needed to mark block as "in-use" */
        break;
      case crops::block_pickup_owner::ekROBOT:
        md()->robot_pickup_event(robot_id);
        md()->first_pickup_time(t);
        break;
      default:
        ER_FATAL_SENTINEL("Unhandled pickup owner %d",
                          rcppsw::as_underlying(owner));
        break;
    } /* switch() */
  }

 private:
  /**
   * \brief Out of sight location blocks are moved to when a robot picks them
   * up, for visualization/rendering purposes.
   */
  struct out_of_sight3D {
    rmath::vector3d rpos{ 1000.0, 1000.0, 0.0 };
    rmath::vector3z dpos{ 1000, 1000, 0 };
  };

  static const out_of_sight3D kOutOfSight;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_SIM_BLOCK3D_HPP_ */
