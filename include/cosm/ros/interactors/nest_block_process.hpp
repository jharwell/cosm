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

#include "rcppsw/types/timestep.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, interactors);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class nest_block_process
 * \ingroup ros interactors
 *
 * \brief Handle's a robot's (possible) nest block process event on a given
 * timestep, updating the real robot state as needed if the conditions for the
 * block drop are met.
 */
template <typename TController, typename TControllerSpecMap>
class nest_block_process
    : public rer::client<nest_block_process<TController, TControllerSpecMap>> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using robot_metrics_manager_type = typename controller_spec::robot_metrics_manager_type;
  using interactor_status_type = typename controller_spec::interactor_status_type;
  using robot_nest_block_process_visitor_type =
      typename controller_spec::robot_nest_block_process_visitor_type;


  explicit nest_block_process(robot_metrics_manager_type* const metrics)
      : ER_CLIENT_INIT("cosm.interactors.ros.nest_block_process"),
        m_metrics_manager(metrics) {}

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
    if (controller.in_nest() && controller.is_carrying_block()) {
      ER_ASSERT(pre_process_check(controller), "Pre-drop check failed");
      process_drop(controller, t);
      return interactor_status_type::ekNEST_BLOCK_PROCESS;
    }
    return interactor_status_type::ekNO_EVENT;
  }

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
  virtual void robot_previsit_hook(TController&) const {}

 private:
  void process_drop(TController& controller, const rtypes::timestep& t) {
    execute_drop(controller, t);
  }

  /**
   * \brief Perform the actual drop in the nest once all preconditions have been
   * satisfied, update both the arena map and the controller, in that order.
   */
  void execute_drop(TController& controller, const rtypes::timestep& t) {
    ER_INFO("Execute: Robot%d, block%d",
            controller.entity_id().v(),
            controller.block()->id().v());
    /*
     * We have to do this asynchronous to the rest of metric collection, because
     * the nest block drop event resets block metrics.
     */
    controller.block()->md()->dest_drop_time(t);
    m_metrics_manager->collect_from_block(controller.block());

    auto to_drop = controller.block_release();

    robot_nest_block_process_visitor_type rdrop_op(to_drop.get(), t);

    /* update bookkeeping */
    robot_previsit_hook(controller);

    /* Actually drop the block */
    rdrop_op.visit(controller);
  }

  bool pre_process_check(const TController& controller) const {
    ER_CHECK(robot_goal_acquired(controller),
             "Robot%d@%s/%s not waiting for nest block drop",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());
    return true;

 error:
    return false;
  }
  /* clang-format off */
  robot_metrics_manager_type* const  m_metrics_manager;
  /* clang-format on */
};

NS_END(interactors, ros, cosm);
