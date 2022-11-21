/**
 * \file base_strategy.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"

#include "cosm/ta/taskable.hpp"
#include "cosm/cosm.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"
#include "cosm/spatial/common/interference_tracker.hpp"
#include "cosm/spatial/common/nest_zone_tracker.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_strategy
 * \ingroup spatial strategy
 *
 * \brief Base class for different strategies that robots can
 * employ when exploring, acquiring goals, etc.
 */
class base_strategy : public cta::taskable {
 public:
  base_strategy(const csfsm::fsm_params*  params, rmath::rng* rng);
  base_strategy(csubsystem::base_saa_subsystem* const saa,
                cspatial::interference_tracker* const inta,
                cspatial::nest_zone_tracker* const nz,
                rmath::rng* rng);

  ~base_strategy(void) override = default;

  base_strategy(const base_strategy&) = delete;
  base_strategy& operator=(const base_strategy&) = delete;

 protected:
  subsystem::base_saa_subsystem* saa(void) const { return m_saa; }
  subsystem::base_saa_subsystem* saa(void) { return m_saa; }
  rmath::rng* rng(void) { return m_rng; }
  rmath::rng* rng(void) const { return m_rng; }
  cspatial::interference_tracker* inta_tracker(void) { return m_inta_tracker; }
  cspatial::interference_tracker* inta_tracker(void) const {
    return m_inta_tracker;
  }
  cspatial::nest_zone_tracker* nz_tracker(void) const { return m_nz_tracker; }
  cspatial::nest_zone_tracker* nz_tracker(void)  { return m_nz_tracker; }

  bool handle_ca(void);
  void phototaxis(void);
  void anti_phototaxis(void);
  void wander(void);
  bool nz_update(void);

 private:
  /* clang-format off */
  subsystem::base_saa_subsystem*  m_saa;
  cspatial::interference_tracker* m_inta_tracker;
  cspatial::nest_zone_tracker*    m_nz_tracker;
  rmath::rng*                     m_rng;
  /* clang-format on */
};

} /* namespace cosm::spatial::strategy */
