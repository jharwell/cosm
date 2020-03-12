/**
 * \file entities_oracle.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ORACLE_ENTITIES_ORACLE_HPP_
#define INCLUDE_COSM_ORACLE_ENTITIES_ORACLE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <boost/variant.hpp>
#include <string>
#include <vector>

#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::repr {
class base_cache;
} /* namespace cosm::foraging::repr */

namespace cosm::oracle::config {
struct entities_oracle_config;
} /* namespace cosm::oracle::config */

NS_START(cosm, oracle);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class entities_oracle
 * \ingroup oracle
 *
 * \brief Repository of perfect knowledge about entities in the arena (blocks,
 * caches, etc).
 */
class entities_oracle final : public rer::client<entities_oracle> {
 public:
  using variant_type = boost::variant<crepr::base_block2D*, cfrepr::base_cache*>;
  using variant_vector_type = std::vector<variant_type>;

  static std::string result_to_string(const variant_vector_type& v);

  explicit entities_oracle(const coconfig::entities_oracle_config* config);

  /**
   * \brief Ask the oracle something.
   *
   * \param query The question to ask. Currently oracles:
   *
   * entities.blocks
   * entities.caches
   *
   * \return The answer to the query. Empty answer if query was ill-formed.
   */
  boost::optional<variant_vector_type> ask(const std::string& query) const;

  void set_blocks(const variant_vector_type& ents) { m_blocks = ents; }
  void set_caches(const variant_vector_type& ents) { m_caches = ents; }

  bool caches_enabled(void) const { return mc_caches; }
  bool blocks_enabled(void) const { return mc_blocks; }

 private:
  /* clang-format off */
  const bool          mc_blocks;
  const bool          mc_caches;

  variant_vector_type m_blocks{};
  variant_vector_type m_caches{};
  /* clang-format on */
};

NS_END(oracle, cosm);

#endif /* INCLUDE_COSM_ORACLE_ENTITIES_ORACLE_HPP_ */
