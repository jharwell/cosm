/**
 * \file strategy_set.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq { class base_acq;}
namespace cosm::spatial::strategy::nest::exit { class base_exit;}
namespace cosm::spatial::strategy::explore { class base_explore;}
namespace cosm::spatial::strategy::blocks::drop { class base_drop;}

namespace cosm::foraging::fsm {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct strategy_set {
  strategy_set(void);
  strategy_set(std::unique_ptr<cssexplore::base_explore> explore_in,
               std::unique_ptr<cssnest::acq::base_acq> nest_acq_in,
               std::unique_ptr<cssnest::exit::base_exit> nest_exit_in,
               std::unique_ptr<cssblocks::drop::base_drop> block_drop);

  ~strategy_set(void);

  strategy_set(strategy_set&& other);

  std::unique_ptr<cssexplore::base_explore> explore{nullptr};
  std::unique_ptr<cssnest::acq::base_acq> nest_acq{nullptr};
  std::unique_ptr<cssnest::exit::base_exit> nest_exit{nullptr};
  std::unique_ptr<cssblocks::drop::base_drop> block_drop{nullptr};
};

} /* namespace cosm::foraging::fsm */
