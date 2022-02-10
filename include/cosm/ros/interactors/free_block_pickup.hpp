/**
 * \file free_block_pickup.hpp
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
#include <memory>
#include <boost/mpl/at.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/real_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, interactors);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class free_block_pickup
 * \ingroup ros interactors
 *
 * \brief Handle's (possible) free block pickup event on a given timestep,
 * updating the real robot state as needed if the conditions for the pickup are
 * met.
 *
 * \tparam TController The type of the robot controller

 * \tparam TControllerSpecMap Compiler time map mapping the type of the
 *         controller to a set of types specifying:
 *
 *         1. The return type of the \p operator() function to use.
 *         2. The type of the visitor to visit the controller in order to
 *            process the block pickup on the robot side (arena map side is
 *            already generic).
 */
template <typename TController, typename TControllerSpecMap>
class free_block_pickup
    : public rer::client<free_block_pickup<TController, TControllerSpecMap>> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using interactor_status_type = typename controller_spec::interactor_status_type;
  using robot_block_pickup_visitor_type =
      typename controller_spec::robot_block_pickup_visitor_type;

  free_block_pickup(void)
      : ER_CLIENT_INIT("cosm.interactors.ros.free_block_pickup") {}

  ~free_block_pickup(void) override = default;

  free_block_pickup(free_block_pickup&&) = default;

  /* Not copy-constructible/assignable by default. */
  free_block_pickup(const free_block_pickup&) = delete;
  free_block_pickup& operator=(const free_block_pickup&) = delete;

  /**
   * \brief Determine if the robot has acquired its goal (a block in this
   * case). This callback is needed because the exact parameters/method of
   * determining when this has occurred are project specific (e.g., checking if
   * the controller's acquisition_goal() function returns a certain integer).
   */
  virtual bool robot_goal_acquired(const TController& controller) const = 0;

  /**
   * \brief Called right before the robot is visited by the block pickup event,
   * in order to provide a hook for derived classes to update controller
   * bookkeeping before the pickup.
   */
  virtual void robot_previsit_hook(TController&) const {}

  /**
   * \brief Handle robot-arena interactions for the specified controller
   * instance on this timestep.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status_type operator()(TController& controller,
                                    const rtypes::timestep& t) {
    if (controller.block_detected()) {
      if (controller.is_carrying_block()) {
        ER_INFO("Ignoring detected block--already carrying block");
      } else {
        process_pickup(controller, t);
        return interactor_status_type::ekARENA_FREE_BLOCK_PICKUP;
      }
    }
    return interactor_status_type::ekNO_EVENT;
  }

 private:
  /**
   * \brief Now that a robot has satisfied its penalty for block pickup,
   * determine if it is still on a free block, and if so execute the block
   * pickup.
   */
  void process_pickup(TController& controller, const rtypes::timestep& t) {
    execute_pickup(controller, t);
  }

  /**
   * \brief Perform the actual picking up of a free block once all
   * preconditions have been satisfied, updating both the arena map and the
   * robot, in that order.
   */
  void execute_pickup(TController& controller,
                      const rtypes::timestep& t) {
    auto block = std::make_unique<crepr::real_block3D>(rutils::color::kBLACK,
                                                       crepr::block_type::ekNONE);

    robot_block_pickup_visitor_type rpickup_op(block.get(),
                                               controller.entity_id(),
                                               t);

    /* update bookkeeping */
    robot_previsit_hook(controller);

    /* actually pickup the block */
    rpickup_op.visit(controller);
  }

  bool pre_process_check(const TController& controller) const {
    ER_CHECK(robot_goal_acquired(controller),
             "Robot%d@%s/%s not waiting for free block pickup",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());
    return true;

  error:
    return false;
  }
};

NS_END(interactors, ros, cosm);

