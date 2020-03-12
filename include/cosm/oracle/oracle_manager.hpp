/**
 * \file oracle_manager.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ORACLE_ORACLE_MANAGER_HPP_
#define INCLUDE_COSM_ORACLE_ORACLE_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/common/common.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::ds {
class arena_map;
} /* namespace cosm::foraging::ds */

namespace cosm::oracle::config {
struct oracle_manager_config;
} /* namespace cosm::oracle::config */

NS_START(cosm, oracle);
class entities_oracle;
class tasking_oracle;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class oracle_manager
 * \ingroup oracle
 *
 * \brief Thin wrapper around the following oracles:
 *
 * - \ref tasking_oracle
 * - \ref entities_oracle
 */
class oracle_manager {
 public:
  explicit oracle_manager(const coconfig::oracle_manager_config* config);

  class entities_oracle* entities_oracle(void) {
    return m_entities.get();
  }
  class tasking_oracle* tasking_oracle(void) {
    return m_tasking.get();
  }
  const class entities_oracle* entities_oracle(void) const {
    return m_entities.get();
  }
  const class tasking_oracle* tasking_oracle(void) const {
    return m_tasking.get();
  }

  /**
   * \brief Because the \ref tasking_oracle requires more than just the \ref
   * tasking_oracle_config to construct, it is null-constructed by default, and
   * the swarm manage is responsible for actually creating it.
   */
  void tasking_oracle(std::unique_ptr<class tasking_oracle> o);

  /**
   * \brief Update all oracles each timestep (if necessary). Should be called
   * from the loop functions before processing any robots for that timestep (at
   * a minimum).
   */
  void update(cfds::arena_map* map);

 private:
  /* clang-format off */
  std::unique_ptr<class entities_oracle> m_entities;
  std::unique_ptr<class tasking_oracle>  m_tasking;
  /* clang-format on */
};

NS_END(oracle, cosm);

#endif /* INCLUDE_COSM_ORACLE_ORACLE_MANAGER_HPP_ */
