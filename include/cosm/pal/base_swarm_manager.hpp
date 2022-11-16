/**
 * \file base_swarm_manager.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ticpp/ticpp.h>

#include <memory>
#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/config/output_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fs = std::filesystem;

NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_swarm_manager
 * \ingroup pal
 *
 * \brief Base class for the harness for support functionality that helps to
 * manage the swarm as it runs, including handling things like:
 *
 * - Metric collection
 *
 * Only core functionality agnostic to the platform on which the swarm control
 * algorithms are being executed is included here.
 */
class base_swarm_manager : public rer::client<base_swarm_manager> {
 public:
  base_swarm_manager(void);

  virtual void init(ticpp::Element& node) = 0;
  virtual void reset(void) = 0;
  virtual void pre_step(void) = 0;
  virtual void post_step(void) = 0;
  virtual void destroy(void) = 0;

  /* Not copy constructable/assignable by default */
  base_swarm_manager(const base_swarm_manager&) = delete;
  const base_swarm_manager& operator=(const base_swarm_manager&) = delete;

  rmath::rng* rng(void) { return m_rng; }

 protected:
  /**
   * \brief Initialize output directories.
   */
  virtual void output_init(const cpconfig::output_config* config) RCPPSW_COLD;

  /**
   * \brief Push the UUID of the swarm manager onto the NDC stack.
   */
  virtual void ndc_uuid_push(void) const = 0;

  /**
   * \brief Pop the last UUID NDC off the stack.
   */
  virtual void ndc_uuid_pop(void) const = 0;

  /**
   * \brief Set the timestamp MDC for the swarm manager.
   */
  virtual void mdc_ts_update(void) const = 0;

  /**
   * \brief Initialize random number generation for loop function use. Currently
   * *NOT* shared between loop functions and robots.
   */
  void rng_init(const rmath::config::rng_config* config) RCPPSW_COLD;

  fs::path output_root(void) const { return m_output_root; }

  void timestep(const rtypes::timestep& t) { m_timestep = t; }
  const rtypes::timestep& timestep(void) const { return m_timestep; }

 private:
  /* clang-format off */
  rtypes::timestep m_timestep{0};
  fs::path         m_output_root{};
  rmath::rng*      m_rng{nullptr};
  /* clang-format on */
};

NS_END(cosm, pal);
