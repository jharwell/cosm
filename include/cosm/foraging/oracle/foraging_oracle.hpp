/**
 * \file foraging_oracle.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>
#include <memory>

#include <boost/mpl/copy.hpp>

#include "rcppsw/common/common.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/oracle/entities_oracle.hpp"
#include "cosm/oracle/aggregate_oracle.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class base_arena_map;
class caching_arena_map;
} /* namespace cosm::arena */

namespace cosm::arena::repr {
class base_cache;
} /* namespace cosm::arena::repr */

namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

namespace cosm::oracle {
class tasking_oracle;
} /* namespace cosm::oracle */

namespace cosm::foraging::oracle {

namespace detail {

using entity_types = rmpl::typelist<crepr::sim_block3D, carepr::base_cache>;
using entity_oracle_types = rmpl::typelist_wrap_apply<entity_types,
                                                      coracle::entities_oracle>;
using tasking_oracle_types = rmpl::typelist<coracle::tasking_oracle>;
using oracle_types = boost::mpl::copy<entity_oracle_types::type,
                                      boost::mpl::back_inserter<tasking_oracle_types>
                                      >::type;

} /* namespace detail */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_oracle
 * \ingroup oracle
 *
 * \brief Oracle/manager for all types of knowledge that can be injected into
 * foraging swarms.
 *
 * These include:
 *
 * - blocks
 * - caches
 * - task execution/interface time estimates (optional; must be adding after
 *   construction)
 */
class foraging_oracle : public coracle::aggregate_oracle<detail::oracle_types> {
 public:
  using blocks_oracle_type = coracle::entities_oracle<crepr::sim_block3D>;
  using caches_oracle_type = coracle::entities_oracle<carepr::base_cache>;
  using tasking_oracle_type = coracle::tasking_oracle;

  static inline const std::string kBlocks = "entities.blocks";
  static inline const std::string kCaches = "entities.caches";
  static inline const std::string kTasks = "tasks";

  explicit foraging_oracle(const coconfig::aggregate_oracle_config* config);

  void tasking_oracle(std::unique_ptr<coracle::tasking_oracle> o);

  const blocks_oracle_type* blocks(void) const {
    return oracle_get<blocks_oracle_type>(kBlocks);
  }

  const caches_oracle_type* caches(void) const {
    return oracle_get<caches_oracle_type>(kBlocks);
  }

  const tasking_oracle_type* tasking(void) const {
    return oracle_get<tasking_oracle_type>(kTasks);
  }

  tasking_oracle_type* tasking(void) {
    return oracle_get<tasking_oracle_type>(kTasks);
  }

  /**
   * \brief Update all oracles each timestep (if necessary). Should be called
   * from the loop functions before processing any robots for that timestep (at
   * a minimum).
   */
  void update(carena::caching_arena_map* map);
  void update(carena::base_arena_map* map);
};

} /* namespace cosm::foraging::oracle */

