/**
 * \file swarm_manager_adaptor.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::pal::ros {

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
  explicit swarm_manager_adaptor(size_t n_robots);
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

#if (RCPPSW_ER >= RCPPSW_ER_ALL)
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

} /* namespace cosm::pal::ros */
