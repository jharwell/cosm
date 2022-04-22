/**
 * \file applicator.hpp
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
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, interactors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class applicator
 * \ingroup interactors
 *
 * \brief Wrapping functor to apply interactions between a robot and the arena
 * each timestep.
 *
 * \tparam TBaseController The type of the base controller class which all
 *                         controllers processed by this class are derived from.

 * \tparam TInteractor The type of the robot-arena interaction_applicatorion
 *                     class. It must (1) take the derived controller type as a
 *                     template argument, (2) define operator(), which takes two
 *                     arguments: the derived controller type as a reference,
 *                     and the current timestep as a constant reference.
 *
 * @note This is not part of the arena or the controller namespace, because it
 * encompasses applying operations to BOTH the arena and controller, so does not
 * fit in either, and gets its own namespace.
 */
template <class TBaseController,
          template <class TDerivedController, class...>
          class TInteractor,
          class... Args>
class applicator {
 public:
  applicator(TBaseController* const controller, const rtypes::timestep& t)
      : mc_timestep(t), m_controller(controller) {}

  /* Not move/copy constructable/assignable by default */
  applicator(const applicator&) = delete;
  const applicator& operator=(const applicator&) = delete;
  applicator(applicator&&) = delete;
  applicator& operator=(applicator&&) = delete;

  template <typename TDerivedController>
  auto operator()(TInteractor<TDerivedController, Args...>& interactor) const
      -> decltype(interactor(std::declval<TDerivedController&>(),
                             std::declval<const rtypes::timestep&>())) {
    return interactor(*static_cast<TDerivedController*>(m_controller),
                      mc_timestep);
  }

 private:
  /* clang-format off */
  const rtypes::timestep mc_timestep;

  TBaseController* const m_controller;
  /* clang-format on */
};

NS_END(interactors, cosm);
