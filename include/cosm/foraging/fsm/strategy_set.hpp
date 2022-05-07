/**
 * \file strategy_set.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include <memory>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq { class base_acq;}
namespace cosm::spatial::strategy::nest::exit { class base_exit;}
namespace cosm::spatial::strategy::explore { class base_explore;}
namespace cosm::spatial::strategy::blocks::drop { class base_drop;}

NS_START(cosm, foraging, fsm);

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

NS_END(fsm, foraging, cosm);
