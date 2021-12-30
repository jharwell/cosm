/**
 * \file pd_adaptor.hpp
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

#ifndef INCLUDE_COSM_PAL_ARGOS_TV_PD_ADAPTOR_HPP_
#define INCLUDE_COSM_PAL_ARGOS_TV_PD_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/tv/switchable_tv_generator.hpp"
#include "cosm/tv/population_dynamics.hpp"
#include "cosm/cosm.hpp"
#include "cosm/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::controller {
class block_carrying_controller;
} /* namespace cosm::controller */

namespace cosm::pal::argos {
class sm_adaptor;
} /* namespace cosm::pal::argos */

namespace cosm::pal::argos::controller {
class adaptor2D;
class adaptorQ3D;
} /* namespace cosm::pal */

NS_START(cosm, pal, argos, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class pd_adaptor
 * \ingroup pal argos tv
 *
 * \brief Adapts \ref ctv::population_dynamics to work within the ARGoS
 * simulator.
 *
 * \tparam TControllertype Must be one of the argos 2D/Q3D controllers, BUT can
 * also be a block carrying controller.
 */
template<typename TController>
class pd_adaptor : public rer::client<pd_adaptor<TController>>,
                   public ctv::population_dynamics {
 public:
  using env_dynamics_type = ctv::env_dynamics<TController>;
  template<typename T>
  using is2D = std::is_base_of<cpargos::controller::adaptor2D, T>;

  template<typename T>
  using isQ3D = std::is_base_of<cpargos::controller::adaptorQ3D, T>;

  static_assert(is2D<TController>::value || isQ3D<TController>::value,
                "TControllerType not derived from ARGoS 2D/Q3D adaptor");
  /**
   * @brief When adding/removing a robot, try this many times to complete the
   * operation.
   */
  static constexpr const size_t kMaxOperationAttempts = 1000;

  pd_adaptor(const ctv::config::population_dynamics_config* config,
             cpargos::sm_adaptor* sm,
             env_dynamics_type *envd,
             const rmath::vector2d& arena_dim,
             rmath::rng* rng);

  pd_adaptor(const pd_adaptor&) = delete;
  const pd_adaptor& operator=(const pd_adaptor&) = delete;

  /* population dynamics overrides */
  op_result robot_kill(void) override;
  op_result robot_add(int max_pop, const rtypes::type_uuid& id) override;
  op_result robot_malfunction(void) override;
  op_result robot_repair(const rtypes::type_uuid& id) override;

  /**
   * \brief Handle any cleanup for the specified controller that needs to be
   * performed on it to maintain simulation fidelity before it is permanently
   * removed from the simulation. Examples:
   *
   * - The robot is currently carrying a block, and you don't want that block to
   *   be permanently lost after the robot is killed.
   */
  virtual void pre_kill_cleanup(TController*) {}

 private:
  TController* malfunction_victim_locate(size_t total_pop) const;
  TController* kill_victim_locate(size_t total_pop) const;
  bool robot_attempt_add(const rtypes::type_uuid& id);

  /* clang-format off */
  const cpargos::sm_adaptor* mc_sm;

  /**
   * \brief The arena dimensions. We already have access to the arena via the
   * \ref cpal::sm_adaptor reference, BUT we don't know what type of arena
   * it is managing, and there is not a clean way that I can see to allow the
   * caller to communicate that information, and doing a series of if/else to
   * figure it out smells bad. So, just pass in the arena dimensions, which is
   * all we need in this class anyway.
   */
  const rmath::vector2d      mc_arena_dim;

  env_dynamics_type*         m_envd;
  rmath::rng*                m_rng;
  cpargos::sm_adaptor*       m_sm;
  /* clang-format on */
};

NS_END(tv, argos, pal, cosm);

#endif /* INCLUDE_COSM_PAL_TV_ARGOS_PD_ADAPTOR_HPP_ */
