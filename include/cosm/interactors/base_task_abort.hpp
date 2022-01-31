/**
 * \file base_task_abort.hpp
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

#ifndef INCLUDE_COSM_INTERACTORS_BASE_TASK_ABORT_HPP_
#define INCLUDE_COSM_INTERACTORS_BASE_TASK_ABORT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include <argos3/core/simulator/entity/floor_entity.h>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, interactors);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class base_task_abort
 * \ingroup interactors
 *
 * \brief Handles a robot's (possible) aborting of its current task on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class base_task_abort
    : public rer::client<base_task_abort<TController, TControllerSpecMap>> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using arena_map_type = typename controller_spec::arena_map_type;
  using envd_type = typename controller_spec::env_dynamics_type;
  using robot_free_block_drop_visitor_type =
      typename controller_spec::robot_free_block_drop_visitor_type;

  base_task_abort(arena_map_type* const map,
                  envd_type* const envd,
                  ::argos::CFloorEntity* const floor)
      : ER_CLIENT_INIT("cosm.interactors.base_task_abort"),
        m_map(map),
        m_envd(envd),
        m_floor(floor) {}

  base_task_abort(base_task_abort&&) = default;

  /* not copy constructible/assignable by default */
  base_task_abort(const base_task_abort&) = delete;
  base_task_abort& operator=(const base_task_abort&) = delete;

  /**
   * \brief Determine if the robot has aborted its task this timestep. This
   * callback is needed because the exact parameters/method of determining when
   * this has occurred are project specific.
   */
  virtual bool robot_task_aborted(const TController& controller) const = 0;

  /**
   * \brief Handle cases in which a robot aborts its current task, and perform
   * any necessary cleanup, such as dropping/distributing a carried block, etc.
   *
   * \param controller The robot to handle task abort for.
   *
   * \return \c TRUE if the robot aborted is current task, \c FALSE otherwise.
   */
  bool operator()(TController& controller) {
    if (!robot_task_aborted(controller)) {
      return false;
    }
    /*
     * If a robot aborted its task and was carrying a block, it needs to (1)
     * drop it so that the block is not left dangling and unusable for the rest
     * of the simulation, (2) update its own internal state.
     */
    if (controller.is_carrying_block()) {
      ER_INFO("Robot%d aborted task while carrying block%d",
              controller.entity_id().v(),
              controller.block()->id().v());
      task_abort_with_block(controller);
    } else {
      ER_INFO("Robot%d aborted task (no block)", controller.entity_id().v());
    }

    m_envd->penalties_flush(controller);
    return true;
  }

 private:
  /**
   * \brief Perform task abort cleanup when the robot was carrying a block as it
   * executed its task, in which the robot just drops the block wherever it
   * happens to be.
   */
  void task_abort_with_block(TController& controller) {
    auto drop_loc = rmath::dvec2zvec(controller.rpos2D(),
                                     m_map->grid_resolution().v());
    auto drop_id = controller.block()->id();

    robot_free_block_drop_visitor_type rdrop_op(controller.block_release(),
                                                drop_loc,
                                                m_map->grid_resolution());

    auto to_drop = m_map->blocks()[drop_id.v()];
    caops::free_block_drop_visitor adrop_op(to_drop,
                                            drop_loc,
                                            m_map->grid_resolution(),
                                            carena::locking::ekNONE_HELD);

    rdrop_op.visit(controller);
    adrop_op.visit(*m_map);

    m_floor->SetChanged();
  } /* perform_block_drop() */

  /* clang-format off */
  arena_map_type* const        m_map;
  envd_type* const             m_envd;
  ::argos::CFloorEntity* const m_floor;
  /* clang-format on */
};

NS_END(interactors, cosm);

#endif /* INCLUDE_COSM_INTERACTORS_BASE_TASK_ABORT_HPP_ */
