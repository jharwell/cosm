/**
 * \file crw.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/spatial/strategy/explore/base_explore.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class crw
 * \ingroup spatial strategy explore
 *
 * \brief Roam around using Correlated Random Walk looking for something until
 * you happen to stumble across it.
 */
class crw : public cssexplore::base_explore,
            public rer::client<crw> {
 public:
  crw(const csfsm::fsm_params* params,
      const cssexplore::config::explore_config* config,
      rmath::rng* rng);

  ~crw(void) override = default;
  crw(const crw&) = delete;
  crw& operator=(const crw&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final {
    m_task_running = true;
  }
  bool task_running(void) const override final { return m_task_running; }

  /**
   * \brief Since we are exploring for something we don't know about, we will
   * never finish (stopping  is handled at a higher level).
   */
  bool task_finished(void) const override final { return false; }
  void task_execute(void) override final;
  void task_reset(void) override final;

  /* prototype overrides */
  std::unique_ptr<base_explore> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<crw>(&params, config(), rng());
  }

 private:
  /* clang-format off */
  bool m_task_running{false};
  /* clang-format on */
};

NS_END(explore, strategy, spatial, cosm);
