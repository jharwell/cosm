/**
 * \file nest_block_process.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <boost/mpl/at.hpp>

#include <argos3/core/simulator/entity/floor_entity.h>

#include "rcppsw/er/client.hpp"

#include "cosm/arena/operations/nest_block_process.hpp"
#include "cosm/tv/temporal_penalty.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::argos::interactors {

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class nest_block_process
 * \ingroup argos interactors
 *
 * \brief Handle's a robot's (possible) nest block process event on a given
 * timestep, updating the robot and the arena map state as needed if the
 * conditions for the block process are met.
 */
template <typename TController, typename TControllerSpecMap>
class nest_block_process
    : public rer::client<nest_block_process<TController, TControllerSpecMap>> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using arena_map_type = typename controller_spec::arena_map_type;
  using penalty_handler_type = typename controller_spec::penalty_handler_type;
  using metrics_manager_type = typename controller_spec::metrics_manager_type;
  using interactor_status_type = typename controller_spec::interactor_status_type;
  using robot_nest_block_process_visitor_type =
      typename controller_spec::robot_nest_block_process_visitor_type;

  nest_block_process(arena_map_type* const map,
                     metrics_manager_type* const metrics,
                     ::argos::CFloorEntity* const floor,
                     penalty_handler_type* handler)
      : ER_CLIENT_INIT("cosm.interactors.argos.nest_block_process"),
        m_metrics_manager(metrics),
        m_map(map),
        m_penalty_handler(handler),
        m_floor(floor) {}

  ~nest_block_process(void) override = default;

  nest_block_process(nest_block_process&&) = default;

  /* Not copy-constructible/assignable by default. */
  nest_block_process(const nest_block_process&) = delete;
  nest_block_process& operator=(const nest_block_process&) = delete;

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
        ER_ASSERT(pre_process_check(controller), "Pre-drop check failed");
        process_drop(controller, t);
        ER_ASSERT(post_process_check(controller), "Post-drop check failed");
        return interactor_status_type::ekNEST_BLOCK_PROCESS;
      }
    } else {
      robot_penalty_init(controller, t, m_penalty_handler);
    }
    return interactor_status_type::ekNO_EVENT;
  }

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


 private:
  void process_drop(TController& controller, const rtypes::timestep& t) {
    /*
     * More than 1 robot can drop a block in a timestep, so we have to
     * search for this robot's controller.
     */
    const auto& p = *m_penalty_handler->penalty_find(controller);
    execute_drop(controller, p, t);
    m_penalty_handler->penalty_remove(p);
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
    m_metrics_manager->collect_from_block(
        static_cast<crepr::sim_block3D*>(controller.block()));

    auto to_drop = controller.block_release();

    /*
     * Safe to index directly even in multi-threaded contexts because the
     * location of blocks within the arena map vector never changes.
     */
    caops::nest_block_process_visitor aproc_op(m_map->blocks()[to_drop->id().v()],
                                               t);
    robot_nest_block_process_visitor_type rdrop_op(
        m_map->blocks()[to_drop->id().v()], t);

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
    aproc_op.visit(*m_map);

    /* Actually drop the block */
    rdrop_op.visit(controller);

    /* The floor texture must be updated */
    m_floor->SetChanged();
  }

    bool pre_process_check(const TController& controller) const {
    /*
     * Can't be an assert(), because sometimes robots wander out of the nest
     * after entering depending on the nest acquisition strategy.
     */
    ER_CONDW(rtypes::constants::kNoUUID !=
                 m_map->robot_in_nest(controller.rpos2D()),
             "Robot%d@%s/%s not in a nest--continuing with nest processing",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());

    ER_CHECK(controller.in_nest(),
             "Robot%d@%s/%s does not think it is in a nest",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());
    ER_CHECK(robot_goal_acquired(controller),
             "Robot%d@%s/%s not waiting for nest block drop",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());
    ER_CHECK(m_penalty_handler->is_serving_penalty(controller),
             "Robot%d@%s/%s not serving drop penalty",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());
    return true;

  error:
    return false;
  }

  bool post_process_check(const TController& controller) const {
    ER_CHECK(!m_penalty_handler->is_serving_penalty(controller),
             "Multiple instances of same controller serving block pickup "
             "penalty");
    return true;

  error:
    return false;
  }

  /* clang-format off */
  metrics_manager_type* const  m_metrics_manager;
  arena_map_type* const        m_map;
  penalty_handler_type* const  m_penalty_handler;
  ::argos::CFloorEntity* const m_floor;
  /* clang-format on */
};

} /* namespace cosm::argos::interactors */

