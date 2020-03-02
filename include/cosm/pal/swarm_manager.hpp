/**
 * \file swarm_manager.hpp
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

#ifndef INCLUDE_COSM_PAL_SWARM_MANAGER_HPP_
#define INCLUDE_COSM_PAL_SWARM_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ext/ticpp/ticpp.h>

#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/hal/hal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class swarm_manager_impl
 * \ingroup cosm pal
 *
 * \brief Base class for the harness for support functionality that helps to
 * manage the swarm as it runs, including handling things like:
 *
 * - Metric collection
 * - Applications of temporal variance
 * - Computing convergence
 */
class swarm_manager_impl : public rer::client<swarm_manager_impl> {
 public:
  swarm_manager_impl(void);


  virtual void init(ticpp::Element& node) = 0;
  virtual void reset(void) = 0;
  virtual void pre_step(void) = 0;
  virtual void post_step(void) = 0;
  virtual void destroy(void) = 0;

  /* Not copy constructable/assignable by default */
  swarm_manager_impl(const swarm_manager_impl&) = delete;
  const swarm_manager_impl& operator=(const swarm_manager_impl&) = delete;

 protected:
  /**
   * \brief Initialize random number generation for loop function use. Currently
   * *NOT* shared between loop functions and robots.
   */
  void rng_init(const rmath::config::rng_config* config) RCSW_COLD;

  const std::string& output_root(void) const { return m_output_root; }

  /**
   * \brief Initialize output directories.
   *
   * \param output_root Absolute/relative path to output root.
   * \param output_dir Directory within the overall output root to use as the
   *                   root directory for THIS experiment's outputs
   */
  void output_init(const std::string& output_root,
                   const std::string& output_dir) RCSW_COLD;

  rmath::rng* rng(void) { return m_rng; }

 private:
  /* clang-format off */
  std::string m_output_root{};
  rmath::rng* m_rng{nullptr};
  /* clang-format on */
};

NS_END(cosm, pal);

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#endif

NS_START(cosm, pal);

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
class swarm_manager : public swarm_manager_impl,
                      public argos::CLoopFunctions,
                      public rer::client<swarm_manager> {
 public:
  swarm_manager(void) : ER_CLIENT_INIT("cosm.pal.swarm_manager") {}

  ~swarm_manager(void) override = default;

  /* Not copy constructable/assignable by default */
  swarm_manager(const swarm_manager&) = delete;
  const swarm_manager& operator=(const swarm_manager&) = delete;

  /* ARGoS hook overrides */
  void Init(ticpp::Element& node) override RCSW_COLD {
    m_floor = &GetSpace().GetFloorEntity();
    init(node);
  }
  void Reset(void) override RCSW_COLD { reset(); }
  void PreStep(void) override { pre_step(); }
  void PostStep(void) override { post_step(); }
  void Destroy(void) override { destroy(); }

  const std::string& led_medium(void) const { return m_led_medium; }

 protected:
#if (LIBRA_ER >= LIBRA_ER_ALL)
  void ndc_push(void) const {
    ER_NDC_PUSH("[t=" + rcppsw::to_string(GetSpace().GetSimulationClock()) +
                "]");
  }
  void ndc_pop(void) const { ER_NDC_POP(); }
#else
  void ndc_push(void) const {}
  void ndc_pop(void) const {}
#endif

  argos::CFloorEntity* floor(void) const { return m_floor; }
  void led_medium(const std::string& s) { m_led_medium = s; }

 private:
  /**
   * \brief The name of the LED medium in ARGoS, for use in destroying caches.
   */
  std::string m_led_medium{};

  /* clang-format on */
  argos::CFloorEntity* m_floor{nullptr};
  /* clang-format off */
};
#else
#error "Unsupported HAL target"
#endif

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_SWARM_MANAGER_HPP_ */
