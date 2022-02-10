/**
 * \file base_strategy.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "rcppsw/utils/color.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"

#include "cosm/ta/taskable.hpp"
#include "cosm/cosm.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"
#include "cosm/spatial/interference_tracker.hpp"
#include "cosm/spatial/nest_zone_tracker.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy);

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
class base_strategy : public cta::taskable,
                      public rpprototype::clonable<base_strategy> {
 public:
  explicit base_strategy(const csfsm::fsm_params*  params,
                         rmath::rng* rng);
  base_strategy(subsystem::saa_subsystemQ3D* const saa,
                cspatial::interference_tracker* const inta,
                cspatial::nest_zone_tracker* const nz,
                rmath::rng* rng);

  ~base_strategy(void) override = default;

  base_strategy(const base_strategy&) = delete;
  base_strategy& operator=(const base_strategy&) = delete;

 protected:
  subsystem::saa_subsystemQ3D* saa(void) const { return m_saa; }
  subsystem::saa_subsystemQ3D* saa(void) { return m_saa; }
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
  void wander(void);
  bool nz_update(void);

 private:
  /* clang-format off */
  subsystem::saa_subsystemQ3D*    m_saa;
  cspatial::interference_tracker* m_inta_tracker;
  cspatial::nest_zone_tracker*    m_nz_tracker;
  rmath::rng*                     m_rng;
  /* clang-format on */
};

NS_END(strategy, spatial, cosm);

