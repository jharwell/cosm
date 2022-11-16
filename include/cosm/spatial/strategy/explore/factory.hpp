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
#include "cosm/spatial/strategy/explore/base_explore.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm);

NS_START(spatial, strategy, explore);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class factory
 * \ingroup spatial strategy explore
 *
 * \brief Factory for creating exploration strategies.
 */
class factory :
    public rpfactory::releasing_factory<cssexplore::base_explore,
                                        std::string, /* key type */
                                        csfsm::fsm_params*,
                                        const cssexplore::config::explore_config*,
                                        rmath::rng*> {
 public:
  static inline const std::string kCRW = "CRW";

  factory(void);
};

NS_END(explore, strategy, spatial, cosm);
