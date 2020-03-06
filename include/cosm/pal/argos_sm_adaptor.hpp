/**
 * \file argos_sm_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_PAL_ARGOS_SM_ADAPTOR_HPP_
#define INCLUDE_COSM_PAL_ARGOS_SM_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/pal/swarm_manager.hpp"
#include "cosm/hal/hal.hpp"

#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::ds {
class arena_map;
} /* namespace cosm::foraging::ds */

namespace cosm::foraging::config {
struct arena_map_config;
} /* namespace cosm:foraging::config */

namespace cosm::vis::config {
struct visualization_config;
} /* namespace cosm:vis::config */

namespace cosm::oracle {
class oracle_manager;
namespace config {
struct oracle_manager_config;
} /* namespace config */
} /* namespace cosm::oracle */

NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_sm_adaptor
 * \ingroup pal
 *
 * \brief Adaptor for \ref swarm_manager to provide an interface for
 * managing swarms within ARGoS.
 */
class argos_sm_adaptor : public swarm_manager,
                         public argos::CLoopFunctions,
                         public rer::client<argos_sm_adaptor> {
 public:
  argos_sm_adaptor(void);
  ~argos_sm_adaptor(void) override;

  /* Not copy constructable/assignable by default */
  argos_sm_adaptor(const argos_sm_adaptor&) = delete;
  const argos_sm_adaptor& operator=(const argos_sm_adaptor&) = delete;

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
  const cfds::arena_map* arena_map(void) const { return m_arena_map.get(); }
  const coracle::oracle_manager* oracle_manager(void) const {
    return m_oracle_manager.get();
  }

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
  cfds::arena_map* arena_map(void) { return m_arena_map.get(); }
  coracle::oracle_manager* oracle_manager(void) {
    return m_oracle_manager.get();
  }
  /**
   * \brief Initialize oracular information injection.
   *
   * \param oraclep Parsed \ref oracle_manager parameters.
   */
  void oracle_init(const coconfig::oracle_manager_config* oraclep) RCSW_COLD;

  /**
   * \brief Initialize the arena contents.
   *
   * \param repo Repository of parsed parameters.
   */
  void arena_map_init(const cfconfig::arena_map_config * aconfig,
                      const cvconfig::visualization_config* vconfig) RCSW_COLD;

  /* clang-format off */
  /**
   * \brief The name of the LED medium in ARGoS, for use in destroying caches.
   */
  std::string                              m_led_medium{};
  argos::CFloorEntity*                     m_floor{nullptr};
  std::unique_ptr<cfds::arena_map>         m_arena_map;
  std::unique_ptr<coracle::oracle_manager> m_oracle_manager;
  /* clang-format on */
};

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_SM_ADAPTOR_HPP_ */
