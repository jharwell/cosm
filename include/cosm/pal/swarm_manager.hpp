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

#include <memory>
#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_manager
 * \ingroup cosm pal
 *
 * \brief Base class for the harness for support functionality that helps to
 * manage the swarm as it runs, including handling things like:
 *
 * - Metric collection
 * - Applications of temporal variance
 * - Computing convergence
 *
 * Only core functionality agnostic to the platform on which the swarm control
 * algorithms are being executed is included here.
 */
class swarm_manager : public rer::client<swarm_manager> {
 public:
  swarm_manager(void);

  virtual void init(ticpp::Element& node) = 0;
  virtual void reset(void) = 0;
  virtual void pre_step(void) = 0;
  virtual void post_step(void) = 0;
  virtual void destroy(void) = 0;

  /* Not copy constructable/assignable by default */
  swarm_manager(const swarm_manager&) = delete;
  const swarm_manager& operator=(const swarm_manager&) = delete;

  rmath::rng* rng(void) { return m_rng; }

 protected:
  /**
   * \brief Initialize random number generation for loop function use. Currently
   * *NOT* shared between loop functions and robots.
   */
  void rng_init(const rmath::config::rng_config* config) RCPPSW_COLD;

  const std::string& output_root(void) const { return m_output_root; }

  /**
   * \brief Initialize output directories.
   *
   * \param output_root Absolute/relative path to output root.
   * \param output_dir Directory within the overall output root to use as the
   *                   root directory for THIS experiment's outputs
   */
  void output_init(const std::string& output_root,
                   const std::string& output_dir) RCPPSW_COLD;

  void timestep(const rtypes::timestep& t) { m_timestep = t; }
  const rtypes::timestep& timestep(void) const { return m_timestep; }

 private:
  /* clang-format off */
  rtypes::timestep                 m_timestep{0};
  std::string                      m_output_root{};
  rmath::rng*                      m_rng{nullptr};
  /* clang-format on */
};

NS_END(cosm, pal);

#endif /* INCLUDE_COSM_PAL_SWARM_MANAGER_HPP_ */
