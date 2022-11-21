/**
 * \file metrics_extract.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <utility>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::controller::operations {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct metrics_extract
 * \ingroup controller operations
 *
 * \brief Functor to perform metric extraction from a controller on each
 * timestep.
 *
 * \tparam TController The type of the controller to extract metrics from.
 * \tparam TAggregator The type of the metrics aggregator to use to extract
 *                         the metrics, derived from \ref
 *                         base_metrics_aggregator.
 *
 * \p TController does not have to be a class template parameter per-se, and
 * could be pushed down to operator(). However, if this is done then the
 * compiler cannot infer the type of the derived controller when trying to apply
 * a specific instantiation of this visitor to a std::variant. So, we encode
 * the necessary type information in the class template parameters.
 */
template <class TController, class TAggregator>
class metrics_extract : public boost::static_visitor<void> {
 public:
  explicit metrics_extract(TAggregator* const agg) : m_agg(agg) {}

  template<typename ...Args>
  void operator()(const TController* const c, Args&& ...args) const {
    m_agg->collect_from_controller(c, std::forward<Args>(args)...);
  }

 private:
  /* clang-format off */
  TAggregator* const m_agg;
  /* clang-format on */
};

} /* namespace cosm::controller::operations */
