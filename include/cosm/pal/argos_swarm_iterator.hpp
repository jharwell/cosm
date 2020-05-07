/**
 * \file argos_swarm_iterator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_PAL_ARGOS_SWARM_ITERATOR_HPP_
#define INCLUDE_COSM_PAL_ARGOS_SWARM_ITERATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/pal/iteration_order.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct argos_swarm_iterator
 * \ingroup pal
 *
 * \brief Iterate over the swarm (robots or controllers), and perform an action
 * within the ARGoS simulator.
 *
 * The selected action can be specified to be performed in static order
 * (single-threaded execution), or in dynamic order (multi-threaded execution)
 * for speed.
 */
struct argos_swarm_iterator {
  /**
   * \brief Iterate through controllers using static ordering.
   *
   * \tparam TRobotType The type of the robot within the ::argos namespace of
   *                    the robots in the swarm.
   * \tparam TControllerType The type of the controller.
   * \tparam order The order of iteration: static or dynamic.
   * \tparam TFunction Type of the lambda callback to use (inferred).
   *
   * \param sm Handle to the \ref cpal::argos_sm_adaptor.
   * \param cb Function to run on each robot in the swarm.
   * \param robot_type Name associated with the robot type within ARGoS.
   */
  template <typename TRobot,
            typename TController,
            iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(iteration_order::ekSTATIC == order)>
  static void controllers(const cpal::argos_sm_adaptor* const sm,
                          const TFunction& cb,
                          const std::string& robot_type) {
    for (auto& [name, robotp] : sm->GetSpace().GetEntitiesByType(robot_type)) {
      auto* robot = ::argos::any_cast<TRobot*>(robotp);
      auto* controller = static_cast<TController*>(
          &robot->GetControllableEntity().GetController());
      cb(controller);
    } /* for(...) */
  }

  /**
   * \brief Iterate through robots using dynamic ordering (OpenMP
   * implementation).
   *
   * \tparam TControllerType The type of the controller.
   * \tparam TFunction Type of the lambda callback (inferred).
   * \tparam order The order of iteration: static or dynamic.
   *
   * \param sm Handle to the \ref cpal::argos_sm_adaptor.
   * \param cb Function to run on each robot in the swarm.
   */
  template <typename TController,
            iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(iteration_order::ekDYNAMIC == order)>
  static void controllers(const cpal::argos_sm_adaptor* const sm,
                          const TFunction& cb) {
    auto wrapper = [&](auto* robot) {
      cb(static_cast<TController*>(&robot->GetController()));
    };
    sm->IterateOverControllableEntities(wrapper);
  }

  /**
   * \brief Iterate through robots using static ordering.
   *
   * \tparam TRobotType The type of the robot within the ::argos namespace of
   *                    the robots in the swarm.
   * \tparam TFunction Type of the lambda callback (inferred).
   * \tparam order The order of iteration: static or dynamic.
   *
   * \param sm Handle to the \ref cpal::argos_sm_adaptor.
   * \param cb Function to run on each robot in the swarm.
   * \param robot_type Name associated with the robot type within ARGoS.
   */
  template <typename TRobot,
            iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(iteration_order::ekSTATIC == order)>
  static void robots(const cpal::argos_sm_adaptor* const sm,
                     const TFunction& cb,
                     const std::string& robot_type) {
    for (auto& [name, robotp] : sm->GetSpace().GetEntitiesByType(robot_type)) {
      auto* robot = ::argos::any_cast<TRobot*>(robotp);
      cb(robot);
    } /* for(...) */
  }

  /**
   * \brief Iterate through robots using dynamic ordering (OpenMP
   * implementation).
   *
   * \tparam TFunction Type of the lambda callback (inferred).
   * \tparam order The order of iteration: static or dynamic.
   *
   * \param sm Handle to the \ref cpal::argos_sm_adaptor.
   * \param cb Function to run on each robot in the swarm.
   */
  template <iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_FUNC(iteration_order::ekDYNAMIC == order)>
  static void robots(const cpal::argos_sm_adaptor* const sm,
                     const TFunction& cb) {
    sm->IterateOverControllableEntities(cb);
  }
};

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_SWARM_ITERATOR_HPP_ */
