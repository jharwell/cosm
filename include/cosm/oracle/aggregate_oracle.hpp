/**
 * \file aggregate_oracle.hpp
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

#ifndef INCLUDE_COSM_ORACLE_AGGREGATE_ORACLE_HPP_
#define INCLUDE_COSM_ORACLE_AGGREGATE_ORACLE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>
#include <memory>
#include <utility>
#include <boost/variant.hpp>

#include "rcppsw/common/common.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/oracle/entities_oracle.hpp"
#include "cosm/oracle/config/aggregate_oracle_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, oracle);
class tasking_oracle;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class aggregate_oracle
 * \ingroup oracle
 *
 * \brief Base class for project oracles (which will contain multiple instances
 * of specific oracles). Enables the injection of perfect knowledge (of one or
 * more types) into the swarm.
 */
template<typename TOracleTypes>
class aggregate_oracle {
 public:
  using oracle_type_variant = typename boost::make_variant_over<
   typename rmpl::typelist_wrap_apply<TOracleTypes,
                                      std::unique_ptr>::type>::type;
  explicit aggregate_oracle(const coconfig::aggregate_oracle_config* config) :
      mc_config(*config) {}

  template<typename TOracleType>
  void oracle_add(const std::string& key,
                  std::unique_ptr<TOracleType> oracle) {
    m_oracles[key] = std::move(oracle);
  }

  template<typename TOracleType>
  const TOracleType* oracle_get(const std::string& key) const {
    auto it = m_oracles.find(key);
    if (m_oracles.end() == it) {
      return nullptr;
    }
    return boost::get<std::unique_ptr<TOracleType>>(it->second).get();
  }

  const coconfig::aggregate_oracle_config* config(void) const { return &mc_config; }

 protected:
  template<typename TOracleType>
  TOracleType* oracle_get(const std::string& key) {
    auto it = m_oracles.find(key);
    if (m_oracles.end() == it) {
      return nullptr;
    }
    return boost::get<std::unique_ptr<TOracleType>>(it->second).get();
  }

 private:
  /* clang-format off */
  const coconfig::aggregate_oracle_config      mc_config;

  std::map<std::string, oracle_type_variant> m_oracles{};
  /* clang-format on */
};

NS_END(oracle, cosm);

#endif /* INCLUDE_COSM_ORACLE_AGGREGATE_ORACLE_HPP_ */
