/**
 * \file factory.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/patterns/factory/factory.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::blocks::drop {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup spatial strategy blocks drop
 *
 * \brief Factory for creating block drop strategies.
 */
class factory :
    public rpfactory::releasing_factory<cssblocks::drop::base_drop,
                                        std::string, /* key type */
                                        csfsm::fsm_params*,
                                        const cssblocks::config::drop_config*,
                                        rmath::rng*> {
 public:
  static inline const std::string kBackup = "backup";
  static inline const std::string kBackupPivot = "backup_pivot";

  factory(void);
};

} /* namespace cosm::spatial::strategy::blocks::drop */
