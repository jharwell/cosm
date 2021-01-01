/**
 * \file tv_manager.hpp
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

#ifndef INCLUDE_COSM_TV_TV_MANAGER_HPP_
#define INCLUDE_COSM_TV_TV_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"
#include "cosm/tv/dynamics_type.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);
class population_dynamics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tv_manager
 * \ingroup tv
 *
 * \brief Orchestrates all application of temporal variance to robot interations
 * with the environment, robotic mechanical functioning, etc.
 *
 * \tparam TEnvDynamics Class which will orchestrate application of
 *         environmental dynamics to the swarm. Project-specific, so must be a
 *         templated type.
 *
 * \tparam TPopulationDynamics Class which will orchestrate population dynamics
 *         within the swarm. Must be derived from \ref population_dynamics.
 */
template <typename TEnvDynamics, typename TPopulationDynamics>
class tv_manager {
 private:
  /**
   * \brief Predicate for detecting if the template types for the class define a
   * method update(const rtypes::timestep&).
   */
  template <typename T>
  using defines_update_type =
      decltype(std::declval<T>().update(std::declval<const rtypes::timestep&>()));

  static_assert(std::is_base_of<population_dynamics, TPopulationDynamics>::value,
                "FATAL: TPopulationDynamics is not derived from "
                "population_dynamics");
  static_assert(rmpl::is_detected<defines_update_type, TEnvDynamics>::value,
                "TEnvDynamics does not define update()");
  static_assert(rmpl::is_detected<defines_update_type, TPopulationDynamics>::value,
                "TPopulationDynamics does not define update()");

 public:
  tv_manager(std::unique_ptr<TEnvDynamics> envd,
             std::unique_ptr<TPopulationDynamics> popd)
      : m_envd(std::move(envd)), m_popd(std::move(popd)) {}

  tv_manager(const tv_manager&) = delete;
  const tv_manager& operator=(const tv_manager&) = delete;

  template <dynamics_type type,
            RCPPSW_SFINAE_FUNC(dynamics_type::ekPOPULATION == type)>
  const TPopulationDynamics* dynamics(void) const {
    return m_popd.get();
  }
  template <dynamics_type type,
            RCPPSW_SFINAE_FUNC(dynamics_type::ekENVIRONMENT == type)>
  const TEnvDynamics* dynamics(void) const {
    return m_envd.get();
  }

  template <dynamics_type type,
            RCPPSW_SFINAE_FUNC(dynamics_type::ekPOPULATION == type)>
  TPopulationDynamics* dynamics(void) {
    return m_popd.get();
  }
  template <dynamics_type type,
            RCPPSW_SFINAE_FUNC(dynamics_type::ekENVIRONMENT == type)>
  TEnvDynamics* dynamics(void) {
    return m_envd.get();
  }

  /**
   * \brief Update the state of all applied variances. Should be called once per
   * timestep.
   */
  void update(const rtypes::timestep& t) {
    m_envd->update(t);
    m_popd->update(t);
  }

 private:
  /* clang-format off */
  std::unique_ptr<TEnvDynamics>        m_envd;
  std::unique_ptr<TPopulationDynamics> m_popd;
  /* clang-format on */
};

NS_END(tv, cosm);

#endif /* INCLUDE_COSM_TV_TV_MANAGER_HPP_ */
