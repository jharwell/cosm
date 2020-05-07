/**
 * \file base_nest_block_drop.hpp
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

#ifndef INCLUDE_COSM_INTERACTORS_BASE_NEST_BLOCK_DROP_HPP_
#define INCLUDE_COSM_INTERACTORS_BASE_NEST_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/mpl/at.hpp>

#include <argos3/core/simulator/entity/floor_entity.h>

#include "cosm/arena/operations/nest_block_drop.hpp"
#include "cosm/tv/temporal_penalty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, interactors);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class base_nest_block_drop
 * \ingroup interactors
 *
 * \brief Handle's a robot's (possible) nest block drop event on a given
 * timestep, updating the robot and the arena map state as needed if the
 * conditions for the drop are met.
 */
template <typename TController, typename TControllerSpecMap>
class base_nest_block_drop
    : public rer::client<base_nest_block_drop<TController, TControllerSpecMap>> {
 public:
  using controller_spec = typename boost::mpl::at<TControllerSpecMap,
                                                  TController>::type;
  using arena_map_type = typename controller_spec::arena_map_type;
  using penalty_handler_type = typename controller_spec::penalty_handler_type;
  using metrics_agg_type = typename controller_spec::metrics_agg_type;
  using interactor_status_type = typename controller_spec::interactor_status_type;
  using robot_nest_block_drop_visitor_type = typename controller_spec::robot_nest_block_drop_visitor_type;

  base_nest_block_drop(arena_map_type* const map,
                       metrics_agg_type* const metrics_agg,
                       argos::CFloorEntity* const floor,
                       penalty_handler_type* handler)
      : ER_CLIENT_INIT("cosm.interactors.base_nest_block_drop"),
        m_floor(floor),
        m_metrics_agg(metrics_agg),
        m_map(map),
        m_penalty_handler(handler) {
  }
  ~base_nest_block_drop(void) override = default;

  base_nest_block_drop(base_nest_block_drop&&) = default;

  /* Not copy-constructible/assignable by default. */
  base_nest_block_drop(const base_nest_block_drop&) = delete;
  base_nest_block_drop& operator=(const base_nest_block_drop&) =
      delete;

  /**
   * \brief If the robot is not currently serving a penalty, then this callback
   * is used to attempt to initialized one. It is needed because the penalty
   * handler type is unknown, and therefore so are the arguments needed for
   * initializing the penalty, beyond the controller and the timestep.
   */
  virtual void robot_penalty_init(const TController& controller,
                                  const rtypes::timestep& t,
                                  penalty_handler_type* handler) = 0;

  /**
   * \brief Determine if the robot has acquired its goal (the nest in this
   * case). This callback is needed because the exact parameters/method of
   * determining when this has occurred are project specific (e.g., checking if
   * the controller's acquisition_goal() function returns a certain integer).
   */
  virtual bool robot_goal_acquired(const TController& controller) const = 0;

  /**
   * \brief Called right before the robot is visited by the block drop event, in
   * order to provide a hook for derived classes to update controller
   * bookkeeping before the pickup.
   */
  virtual void robot_previsit_hook(TController&,
                                   const ctv::temporal_penalty&) const {}

  /**
   * \brief Handler robot-arena interactions for the specified controller
   * instance on this timestep.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status_type operator()(TController& controller,
                                    const rtypes::timestep& t) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        process_drop(controller, t);
        return interactor_status_type::ekNEST_BLOCK_DROP;
      }
    } else {
      robot_penalty_init(controller, t, m_penalty_handler);
    }
    return interactor_status_type::ekNO_EVENT;
  }

 private:
  /**
   * \brief Now that a robot has satisfied its penalty for nest drop, perform a
   * few sanity checks and then execute the block drop.
   */
  void process_drop(TController& controller, const rtypes::timestep& t) {
    ER_ASSERT(controller.in_nest(), "Controller not in nest");
    ER_ASSERT(robot_goal_acquired(controller),
              "Controller not waiting for nest block drop");
    ER_ASSERT(m_penalty_handler->is_serving_penalty(controller),
              "Controller not serving drop penalty");
    /*
     * More than 1 robot can drop a block in a timestep, so we have to
     * search for this robot's controller.
     */
    const auto& p = *m_penalty_handler->penalty_find(controller);
    execute_drop(controller, p, t);
    m_penalty_handler->penalty_remove(p);
    ER_ASSERT(!m_penalty_handler->is_serving_penalty(controller),
              "Multiple instances of same controller serving drop penalty");
  }

  /**
   * \brief Perform the actual drop in the nest once all preconditions have been
   * satisfied, update both the arena map and the controller, in that order.
   */
  void execute_drop(TController& controller,
                    const ctv::temporal_penalty& penalty,
                    const rtypes::timestep& t) {
    /*
     * We have to do this asynchronous to the rest of metric collection, because
     * the nest block drop event resets block metrics.
     */
    controller.block()->md()->dest_drop_time(t);
    m_metrics_agg->collect_from_block(controller.block());

    rtypes::type_uuid id = controller.block()->id();
    caops::nest_block_drop_visitor adrop_op(controller.block_release(), t);

    /*
     * Safe to index directly even in multi-threaded contexts because the
     * location of blocks within their arena map vector never changes.
     */
    robot_nest_block_drop_visitor_type rdrop_op(m_map->blocks()[id.v()], t);

    /* update bookkeeping */
    robot_previsit_hook(controller, penalty);

    /*
     * Order of visitation must be:
     *
     * 1. Arena map
     * 2. Controller
     *
     * In order for the event to process properly.
     */
    /* Update arena map state due to a block nest drop */
    adrop_op.visit(*m_map);

    /* Actually drop the block */
    rdrop_op.visit(controller);

    /* The floor texture must be updated */
    m_floor->SetChanged();
  }

  /* clang-format off */
  argos::CFloorEntity* const  m_floor;
  metrics_agg_type* const     m_metrics_agg;
  arena_map_type* const       m_map;
  penalty_handler_type* const m_penalty_handler;
  /* clang-format on */
};

NS_END(interactors, cosm);

#endif /* INCLUDE_COSM_INTERACTORS_BASE_NEST_BLOCK_DROP_HPP_ */
