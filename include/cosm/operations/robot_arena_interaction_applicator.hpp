/**
 * \file robot_arena_interaction_applicator.hpp
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
#ifndef INCLUDE_COSM_OPERATIONS_ROBOT_ARENA_INTERACTION_APPLICATOR_HPP_
#define INCLUDE_COSM_OPERATIONS_ROBOT_ARENA_INTERACTION_APPLICATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_arena_interaction_applicator
 * \ingroup operations
 *
 * \brief Wrapping functor to apply interactions between a robot and the arena
 * each timestep.
 *
 * \tparam TBaseControllerType The type of the base controller class which all
 *                             controllers processed by this class are derived
 *                             from.

 * \tparam TInteractorType The type of the robot-arena interaction_applicatorion
 *                         class. It must (1) take the derived controller type
 *                         as a template argument, (2) define operator(), which
 *                         takes two arguments: the derived controller type as a
 *                         reference, and the current timestep as a constant
 *                         reference.
 *
 * @note This is not part of the arena or the controller namespace, because it
 * encompasses applying operations to BOTH the arena and controller, so does not
 * fit in either.
 */
template <class TBaseControllerType,
          template <class TDerivedControllerType> class TInteractorType>
class robot_arena_interaction_applicator {
 public:
  robot_arena_interaction_applicator(TBaseControllerType* const controller,
                           const rtypes::timestep& t)
      : mc_timestep(t), m_controller(controller) {}

  template <typename TDerivedControllerType>
  auto operator()(TInteractorType<TDerivedControllerType>& interactor) const -> decltype(interactor(std::declval<TDerivedControllerType&>(),
                                                                                                    std::declval<const rtypes::timestep&>())) {
    return interactor(*static_cast<TDerivedControllerType*>(m_controller),
                      mc_timestep);
  }

 private:
  /* clang-format off */
  const rtypes::timestep     mc_timestep;

  TBaseControllerType* const m_controller;
  /* clang-format on */
};

NS_END(operations, cosm);

#endif /* INCLUDE_COSM_OPERATIONS_ROBOT_ARENA_INTERACTION_APPLICATOR_HPP_ */
