/**
 * \file aggregate_oracle.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "rcppsw/common/common.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/oracle/config/aggregate_oracle_config.hpp"
#include "cosm/oracle/entities_oracle.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::oracle {
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
template <typename TOracleTypes>
class aggregate_oracle {
 public:
  using oracle_type_variant = typename boost::make_variant_over<
      typename rmpl::typelist_wrap_apply<TOracleTypes, std::unique_ptr>::type>::type;
  explicit aggregate_oracle(const coconfig::aggregate_oracle_config* config)
      : mc_config(*config) {}

  template <typename TOracle>
  void oracle_add(const std::string& key, std::unique_ptr<TOracle> oracle) {
    m_oracles[key] = std::move(oracle);
  }

  template <typename TOracle>
  const TOracle* oracle_get(const std::string& key) const {
    auto it = m_oracles.find(key);
    if (m_oracles.end() == it) {
      return nullptr;
    }
    return boost::get<std::unique_ptr<TOracle>>(it->second).get();
  }

  const coconfig::aggregate_oracle_config* config(void) const {
    return &mc_config;
  }

 protected:
  template <typename TOracle>
  TOracle* oracle_get(const std::string& key) {
    auto it = m_oracles.find(key);
    if (m_oracles.end() == it) {
      return nullptr;
    }
    return boost::get<std::unique_ptr<TOracle>>(it->second).get();
  }

 private:
  /* clang-format off */
  const coconfig::aggregate_oracle_config      mc_config;

  std::map<std::string, oracle_type_variant> m_oracles{};
  /* clang-format on */
};

} /* namespace cosm::oracle */
