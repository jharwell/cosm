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

#ifndef INCLUDE_COSM_FSM_EXPSTRAT_BASE_EXPSTRAT_HPP_
#define INCLUDE_COSM_FSM_EXPSTRAT_BASE_EXPSTRAT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/metrics/collision_metrics.hpp"
#include "rcppsw/ta/taskable.hpp"
#include "rcppsw/utils/color.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm);

namespace subsystem {
class saa_subsystem2D;
} /* namespace subsystem */

NS_START(fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_expstrat
 * \ingroup fsm expstrat
 *
 * \brief Base class for different exploration behaviors that controller can
 * exhibit when looking for stuff.
 */
class base_expstrat : public fsm::metrics::collision_metrics,
                      public rta::taskable {
 public:
  struct params {
    explicit params(subsystem::saa_subsystem2D* const saa_in)
        : saa(saa_in) {}

    subsystem::saa_subsystem2D* const saa;
  };

  explicit base_expstrat(params* const p)
      : base_expstrat{p->saa} {}

  explicit base_expstrat(subsystem::saa_subsystem2D* const saa)
      : m_saa(saa) {}

  ~base_expstrat(void) override = default;

  base_expstrat(const base_expstrat&) = delete;
  base_expstrat& operator=(const base_expstrat&) = delete;

 protected:
  subsystem::saa_subsystem2D* saa(void) const { return m_saa; }
  subsystem::saa_subsystem2D* saa(void) { return m_saa; }

 private:
  /* clang-format off */
  subsystem::saa_subsystem2D* m_saa;
  /* clang-format on */
};

NS_END(expstrat, fsm, cosm);

#endif /* INCLUDE_COSM_FSM_EXPSTRAT_BASE_EXPSTRAT_HPP_ */
