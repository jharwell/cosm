/**
 * \file base_drop.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/base_strategy.hpp"

#include "cosm/spatial/strategy/blocks/config/drop_config.hpp"
#include "cosm/spatial/strategy/blocks/metrics/drop_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::blocks::drop {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_drop
 * \ingroup spatial strategy blocks drop
 *
 * \brief Base class for block drop strategies, to make doing experiments with
 * real robots where they actually have to do SOMETHING to drop a carried block
 * easier.
 */

class base_drop : public csstrategy::base_strategy,
                  public rpprototype::clonable<base_drop>,
                  public cssblocks::metrics::drop_metrics {
 public:
  base_drop(const csfsm::fsm_params* params,
            const cssblocks::config::drop_config*,
            rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  base_drop(const base_drop&) = delete;
  base_drop& operator=(const base_drop&) = delete;
  base_drop(base_drop&&) = delete;
  base_drop& operator=(base_drop&&) = delete;

  const cssblocks::drop::base_drop* block_drop_strategy(void) const override {
    return this;
  }

  const cssblocks::config::drop_config* config(void) const { return &mc_config; }

 private:
  /* clang-formatt off */
  const cssblocks::config::drop_config mc_config;
  /* clang-formatt on */
};

} /* namespace cosm::spatial::strategy::blocks::drop */
