/**
 * \file entities_oracle.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <numeric>
#include <string>
#include <vector>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class entities_oracle
 * \ingroup oracle
 *
 * \brief Repository of perfect knowledge about entities the swarm may encounter
 * (blocks, caches, etc).
 */
template <class TEntity>
class entities_oracle {
 public:
  using knowledge_type = std::vector<TEntity*>;

  /**
   * \brief Get a string representation of the oracle's knowledge. Entities are
   * prefixed by the specified prefix.
   */
  static std::string knowledge_to_string(const std::string& prefix,
                                         const knowledge_type& v) {
    auto lambda = [&](const std::string& a, const auto& ent) {
      return a + prefix + rcppsw::to_string(ent->id()) + ",";
    };
    return std::accumulate(v.begin(), v.end(), std::string(), lambda);
  }

  /**
   * \brief Ask the oracle to return its knowledge about the entities it
   * currently holds.
   */
  const knowledge_type& ask(void) const { return m_knowledge; }

  void set_knowledge(const knowledge_type& k) { m_knowledge = k; }

 private:
  /* clang-format off */
  knowledge_type m_knowledge{};
  /* clang-format on */
};

NS_END(oracle, cosm);
