/**
 * \file swarm_manager_adaptor.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <ticpp/ticpp.h>

#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/pal/base_swarm_manager.hpp"
#include "cosm/repr/block_variant.hpp"
#include "cosm/argos/block_embodiment_variant.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena::config {
struct arena_map_config;
} // namespace cosm::arena::config

namespace cosm::repr::config {
struct nests_config;
} /* namespace cosm::repr::config */
namespace cosm::arena {
class base_arena_map;
class caching_arena_map;
} // namespace cosm::arena

namespace cosm::argos::vis::config {
struct visualization_config;
} // namespace cosm::vis::config

NS_START(cosm, pal, argos);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_manager_adaptor
 * \ingroup pal argos
 *
 * \brief Adaptor for \ref swarm_manager to provide an interface for
 * managing swarms within ARGoS.
 */
class swarm_manager_adaptor : public cpal::base_swarm_manager,
                              public ::argos::CLoopFunctions,
                              public rer::client<swarm_manager_adaptor> {
 public:
  swarm_manager_adaptor(void);
  ~swarm_manager_adaptor(void) override;

  /* Not copy constructable/assignable by default */
  swarm_manager_adaptor(const swarm_manager_adaptor&) = delete;
  const swarm_manager_adaptor& operator=(const swarm_manager_adaptor&) = delete;

  /* swarm_manager overrides */
  void init(ticpp::Element&) override;
  void pre_step(void) override;
  void reset(void) override {}
  void post_step(void) override {}
  void destroy(void) override {}

  /* ARGoS hook overrides */
  void Init(ticpp::Element& node) override RCPPSW_COLD { init(node); };
  void Reset(void) override RCPPSW_COLD { reset(); }
  void PreStep(void) override { pre_step(); }
  void PostStep(void) override { post_step(); }
  void Destroy(void) override { destroy(); }
  ::argos::CColor GetFloorColor(const ::argos::CVector2& pos) override;

  const std::string& led_medium(void) const { return m_led_medium; }
  const carena::base_arena_map* arena_map(void) const {
    return m_arena_map.get();
  }
  ::argos::CFloorEntity* floor(void) const { return m_floor; }

 protected:
#if (LIBRA_ER >= LIBRA_ER_ALL)
  void ndc_uuid_push(void) const override final {
    ER_NDC_PUSH("[argos_sm]");
  }
  void ndc_uuid_pop(void) const override final { ER_NDC_POP(); }

  void mdc_ts_update(void) const override final {
    ER_MDC_RM("time");
    ER_MDC_ADD("time",
               "[t=" + rcppsw::to_string(GetSpace().GetSimulationClock()) + "]");
  }
#else
  void ndc_uuid_push(void) const override {}
  void ndc_uuid_pop(void) const override {}
  void mdc_ts_update(void) const override {}
#endif

  void led_medium(const std::string& s) { m_led_medium = s; }
  carena::base_arena_map* arena_map(void) { return m_arena_map.get(); }

  template <typename TArenaMap>
  void arena_map_create(const caconfig::arena_map_config* aconfig) RCPPSW_COLD;

  /**
   * \brief Initialize the arena contents.
   */
  void arena_map_init(const cavis::config::visualization_config* vconfig,
                      const crepr::config::nests_config* nconfig) RCPPSW_COLD;

 private:
  /* clang-format off */
  /**
   * \brief The name of the LED medium in ARGoS, for use in destroying caches.
   */
  std::string                             m_led_medium{};
  ::argos::CFloorEntity*                  m_floor{nullptr};
  std::unique_ptr<carena::base_arena_map> m_arena_map{};
  /* clang-format on */
};

NS_END(argos, pal, cosm);

