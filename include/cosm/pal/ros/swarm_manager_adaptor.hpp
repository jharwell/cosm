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
#include "rcppsw/types/timestep.hpp"

#include "cosm/pal/base_swarm_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, ros);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_manager_adaptor
 * \ingroup pal ros
 *
 * \brief Adaptor for \ref base_swarm_manager to provide an interface for
 * managing swarms within ROS.
 */
class swarm_manager_adaptor : public cpal::base_swarm_manager,
                              public rer::client<swarm_manager_adaptor> {
 public:
  swarm_manager_adaptor(size_t n_robots);
  ~swarm_manager_adaptor(void) override = default;

  /* Not copy constructable/assignable by default */
  swarm_manager_adaptor(const swarm_manager_adaptor&) = delete;
  const swarm_manager_adaptor& operator=(const swarm_manager_adaptor&) = delete;

  /**
   * \brief Is it time for the experiment to end?
   */
  virtual bool experiment_finished(void) const = 0;

  /* swarm_manager overrides */
  void init(ticpp::Element&) override {}
  void pre_step(void) override {}
  void reset(void) override {}
  void post_step(void) override {}
  void destroy(void) override {}

 protected:
  size_t swarm_size(void) const { return mc_n_robots; }

#if (LIBRA_ER >= LIBRA_ER_ALL)
  void ndc_uuid_push(void) const override final {
    ER_NDC_PUSH("[ros_sm]");
  }
  void ndc_uuid_pop(void) const override final { ER_NDC_POP(); }
  void mdc_ts_update(void) const override final {
    ER_MDC_RM("time");
    ER_MDC_ADD("time", "[t=" + rcppsw::to_string(timestep()) + "]");
  }
#else
  void ndc_uuid_push(void) const override final {}
  void ndc_uuid_pop(void) const override final {}
  void mdc_ts_update(void) const override final {}
#endif

 private:
  /* clang-format off */
  const size_t mc_n_robots;
  /* clang-format on */
};

NS_END(ros, pal, cosm);
