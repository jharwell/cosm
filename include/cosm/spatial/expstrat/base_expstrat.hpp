/**
 * \file base_expstrat.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_EXPSTRAT_BASE_EXPSTRAT_HPP_
#define INCLUDE_COSM_SPATIAL_EXPSTRAT_BASE_EXPSTRAT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"

#include "cosm/spatial/metrics/interference_metrics.hpp"
#include "cosm/spatial/interference_tracker.hpp"

#include "cosm/ta/taskable.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {
class saa_subsystemQ3D;
} /* namespace subsystem */

NS_START(cosm, spatial, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_expstrat
 * \ingroup spatial expstrat
 *
 * \brief Base class for different exploration behaviors that controller can
 * exhibit when looking for stuff.
 */
class base_expstrat : public csmetrics::interference_metrics,
                      public cta::taskable,
                      public rpprototype::clonable<base_expstrat> {
 public:
  struct params {
    params(subsystem::saa_subsystemQ3D* const saa_in, rmath::rng* rng_in)
        : saa(saa_in), rng(rng_in) {}

    subsystem::saa_subsystemQ3D* const saa;
    rmath::rng* rng;
  };

  explicit base_expstrat(params* const p);
  base_expstrat(subsystem::saa_subsystemQ3D* const saa, rmath::rng* rng);

  ~base_expstrat(void) override = default;

  base_expstrat(const base_expstrat&) = delete;
  base_expstrat& operator=(const base_expstrat&) = delete;

 protected:
  subsystem::saa_subsystemQ3D* saa(void) const { return m_saa; }
  subsystem::saa_subsystemQ3D* saa(void) { return m_saa; }
  rmath::rng* rng(void) { return m_rng; }
  rmath::rng* rng(void) const { return m_rng; }
  const cspatial::interference_tracker* inta_tracker(void) const {
    return &m_inta_tracker;
  }
  cspatial::interference_tracker* inta_tracker(void) {
    return &m_inta_tracker;
  }

 private:
  /* clang-format off */
  subsystem::saa_subsystemQ3D*   m_saa;
  rmath::rng*                    m_rng;
  cspatial::interference_tracker m_inta_tracker;
  /* clang-format on */

 public:
  /* collision metrics */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exp_interference, m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(entered_interference, m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exited_interference, m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_duration, m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_loc3D, m_inta_tracker, const)
};

NS_END(expstrat, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_EXPSTRAT_BASE_EXPSTRAT_HPP_ */
