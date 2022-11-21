/**
 * \file swarm_iterator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/pal/argos/swarm_manager_adaptor.hpp"
#include "cosm/pal/iteration_order.hpp"
#include "cosm/hal/robot.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, argos);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct swarm_iterator
 * \ingroup pal argos
 *
 * \brief Iterate over the swarm (robots or controllers), and perform an action
 * within the ARGoS simulator.
 *
 * The selected action can be specified to be performed in static order
 * (single-threaded execution), or in dynamic order (multi-threaded execution)
 * for speed.
 */
struct swarm_iterator {
  /**
   * \brief Iterate through controllers using static ordering.
   *
   * \tparam TControllerType The type of the controller.
   * \tparam order The order of iteration: static or dynamic.
   * \tparam TFunction Type of the lambda callback to use (inferred).
   *
   * \param sm Handle to the \ref cpargos::swarm_manager_adaptor.
   * \param cb Function to run on each robot in the swarm.
   * \param robot_type Name associated with the robot type within ARGoS.
   */
  template <typename TController,
            iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_DECLDEF(iteration_order::ekSTATIC == order)>
  static void controllers(const cpargos::swarm_manager_adaptor* const sm,
                          const TFunction& cb,
                          const std::string& robot_type) {
    for (auto& [name, robotp] : sm->GetSpace().GetEntitiesByType(robot_type)) {
      auto* robot = ::argos::any_cast<chal::robot*>(robotp);
      auto* controller = static_cast<TController*>(
          &robot->GetControllableEntity().GetController());
      cb(controller);
    } /* for(...) */
  }

  /**
   * \brief Iterate through robots using dynamic ordering.
   *
   * \tparam TControllerType The type of the controller.
   * \tparam TFunction Type of the lambda callback (inferred).
   * \tparam order The order of iteration: static or dynamic.
   *
   * \param sm Handle to the \ref cpargos::swarm_manager_adaptor.
   * \param cb Function to run on each robot in the swarm.
   */
  template <typename TController,
            iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_DECLDEF(iteration_order::ekDYNAMIC == order)>
  static void controllers(const cpargos::swarm_manager_adaptor* const sm,
                          const TFunction& cb) {
    auto wrapper = [&](auto* robot) {
      cb(static_cast<TController*>(&robot->GetController()));
    };
    sm->IterateOverControllableEntities(wrapper);
  }

  /**
   * \brief Iterate through robots using static ordering.
   *
   * \tparam TFunction Type of the lambda callback (inferred).
   * \tparam order The order of iteration: static or dynamic.
   *
   * \param sm Handle to the \ref cpargos::swarm_manager_adaptor.
   * \param cb Function to run on each robot in the swarm.
   * \param robot_type Name associated with the robot type within ARGoS.
   */
  template <iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_DECLDEF(iteration_order::ekSTATIC == order)>
  static void robots(const cpargos::swarm_manager_adaptor* const sm,
                     const TFunction& cb,
                     const std::string& robot_type) {
    for (auto& [name, robotp] : sm->GetSpace().GetEntitiesByType(robot_type)) {
      auto* robot = ::argos::any_cast<chal::robot*>(robotp);
      cb(robot);
    } /* for(...) */
  }

  /**
   * \brief Iterate through robots using dynamic ordering.
   *
   * \tparam TFunction Type of the lambda callback (inferred).
   * \tparam order The order of iteration: static or dynamic.
   *
   * \param sm Handle to the \ref cpargos::swarm_manager_adaptor.
   * \param cb Function to run on each robot in the swarm.
   */
  template <iteration_order order,
            typename TFunction,
            RCPPSW_SFINAE_DECLDEF(iteration_order::ekDYNAMIC == order)>
  static void robots(const cpargos::swarm_manager_adaptor* const sm,
                     const TFunction& cb) {
    sm->IterateOverControllableEntities(cb);
  }
};

NS_END(argos, pal, cosm);

