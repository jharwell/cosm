/**
 * \file collector_registerer.hpp
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

#ifndef INCLUDE_COSM_METRICS_COLLECTOR_REGISTERER_HPP_
#define INCLUDE_COSM_METRICS_COLLECTOR_REGISTERER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <filesystem>
#include <set>
#include <string>
#include <tuple>
#include <utility>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/mpl/identity.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/cosm.hpp"
#include "cosm/metrics/base_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, metrics, detail);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template <typename>
class collector_registerer_impl;

/**
 * \class collector_registerer_impl
 * \ingroup metrics detail
 *
 * \brief After enabled collectors have been parsed from an XML input file,
 * register the enabled collectors with a \ref base_metrics_aggregator or a
 * class derived from it.
 *
 * This class is a partial specialization which specializes the generic
 * collector_registerer_impl, which can take an arbitrary # of types, to take a
 * std::tuple which in turn takes an arbitrary # of types. This is a better
 * design than simply having a class that takes an arbitrary # of types, because
 * it makes it clearer that the template parameters are considered a single
 * unit, and are NOT part of the constructor arguments for the class.
 */
template <typename... Ts>
class collector_registerer_impl<std::tuple<Ts...>>
    : public rer::client<collector_registerer_impl<std::tuple<Ts...>>> {
 public:
  /**
   * \brief Each entry in the set of collectors that CAN be created (if they are
   * actually created dependent on configuration) has:
   *
   * - The typeid of the collector, so that functions templated on collector
   *   type can figure out the correct item in the set to read from.
   *
   * - The name of the collector in the input src (e.g. the XML attribute name).
   *
   * - The scoped name of the collector that will be used to refer to the
   *   created collector during simulation.
   *
   * - The set of output modes that are valid for the collector.
   */
  using set_value_type =
      std::tuple<std::type_index, std::string, std::string, rmetrics::output_mode>;

  /**
   * \brief Comparator for \ref set_value_type objects within the \ref
   * creatable_set.
   */
  struct set_comparator {
    bool operator()(const set_value_type& lhs, const set_value_type& rhs) const {
      return std::get<0>(lhs) < std::get<0>(rhs);
    }
  };

  /**
   * \brief Set of \ref set_value_types in which duplicates are allowed, because
   * when we compare elements, we only use the typeid as the key, which can be
   * the same between collectors, even if the other parts of each element are
   * different.
   */
  using creatable_set = std::multiset<set_value_type, set_comparator>;

  /**
   * \brief A collector is constructible using the expected function
   * arguments. These collectors always use \ref
   * rmetrics::output_mode::ekAPPEND.
   */
  template <typename T>
  using expected_constructible =
      std::is_constructible<T, const std::string&, const rtypes::timestep&>;

  /**
   * \brief Some metrics collectors do not require the collection interval
   * argument to their constructor, as they MUST be gathered every timestep,
   * regardless of configuration. These collectors always use \ref
   * rmetrics::output_mode::ekAPPEND.
   */
  template <typename T>
  using constructible_without_collect_interval =
      std::is_constructible<T, const std::string&>;

  /**
   * \brief Some metrics collectors (e.g. \ref bi_tdgraph_metrics_collector)
   * require an additional integer as an argument to their constructor. This
   * requirement for additional specialized arguments required by some
   * collectors is satisfied in a general way by forwarding the \p
   * TExtraArgsTuple to the constructor for the type.
   *
   * These collectors must always use \ref rmetrics::output_mode::ekAPPEND.
   */
  template <typename TCollector>
  using constructible_with_extra_args =
      std::is_constructible<TCollector,
                            const std::string&,
                            const rtypes::timestep&,
                            Ts...>;

  template <typename TCollector>
  using constructible_with_mode_and_extra_args =
      std::is_constructible<TCollector,
                            const std::string&,
                            const rtypes::timestep&,
                            rmetrics::output_mode,
                            Ts...>;
  /**
   * \brief Initialize the registerer.
   *
   * \param config Metrics configuration, specifying which metrics should be
   *               collected.
   *
   * \param agg The metrics aggregator to register the collectors with.
   *
   * \param create_set Definitions for all the possible collectors to create.
   * \param decomposition_depth The height of the complete binary tree formed by
   *                           the task decomposition graph.
   *
   * Both of the maps are necessary to provide an input src agnostic means of
   * mapping collectors to run-time categories, so that this class is general
   * purpose and not tied to a specific input format.
   */
  template <typename... Args>
  collector_registerer_impl(
      const cmconfig::metrics_config* const config,
      const creatable_set& create_set,
      base_metrics_aggregator* const agg,
      const std::tuple<Args...>& extra_args = std::tuple<Args...>())
      : ER_CLIENT_INIT("cosm.metrics.collector_registerer"),
        mc_extra_args(extra_args),
        mc_config(config),
        mc_create_set(create_set),
        m_agg(agg) {}
  collector_registerer_impl(const collector_registerer_impl&) = default;
  collector_registerer_impl operator=(const collector_registerer_impl&) = delete;

  template <typename TCollectorWrap>
  void operator()(const TCollectorWrap&) const {
    std::type_index id(typeid(typename TCollectorWrap::type));

    auto key = set_value_type(id, "", "", rmetrics::output_mode::ekNONE);
    auto range = mc_create_set.equal_range(key);

    ER_ASSERT(mc_create_set.end() != mc_create_set.find(key),
              "Unknown collector: type_index='%s'",
              id.name());

    /*
     * Multiple collectors of the same type can be registered as different
     * scoped/runtime names, so we need to iterate.
     */
    for (auto it = range.first; it != range.second; ++it) {
      if (auto init =
              collector_pre_initialize(std::get<1>(*it), std::get<3>(*it))) {
        bool ret = do_register<TCollectorWrap>(
            std::get<2>(*it), init->fpath, init->output_interval, init->mode);
        if (!ret) {
          ER_WARN("Collector with scoped_name='%s' already exists!",
                  std::get<2>(*it).c_str());
        } else {
          ER_INFO(
              "Metrics enabled: "
              "xml_name='%s',scoped_name='%s',fpath_stem=%s,output_interval=%d,"
              "mode=%x",
              std::get<1>(*it).c_str(),
              std::get<2>(*it).c_str(),
              init->fpath.c_str(),
              init->output_interval.v(),
              rcppsw::as_underlying(init->mode));
        }
      }
    } /* for(it..) */
  }

 private:
  struct pre_init_ret_type {
    fs::path fpath;
    rtypes::timestep output_interval;
    rmetrics::output_mode mode;
  };

  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(
                expected_constructible<typename TCollectorWrap::type>::value)>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath,
                   const rtypes::timestep& interval,
                   rmetrics::output_mode mode) const {
    m_agg->collector_preregister(scoped_name, mode);
    return m_agg->collector_register<typename TCollectorWrap::type>(scoped_name,
                                                                    fpath,
                                                                    interval);
  }

  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(constructible_without_collect_interval<
                               typename TCollectorWrap::type>::value)>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath,
                   const rtypes::timestep&,
                   rmetrics::output_mode mode) const {
    m_agg->collector_preregister(scoped_name, mode);
    return m_agg->collector_register<typename TCollectorWrap::type>(scoped_name,
                                                                    fpath);
  }
  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(constructible_with_extra_args<
                                   typename TCollectorWrap::type>::value &&
                               (sizeof...(Ts) > 0))>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath,
                   const rtypes::timestep& interval,
                   rmetrics::output_mode mode) const {
    m_agg->collector_preregister(scoped_name, mode);
    auto targs = std::tuple_cat(std::make_tuple(scoped_name, fpath, interval),
                                mc_extra_args);
    auto lambda = [&](auto&&... args) {
      return m_agg->collector_register<typename TCollectorWrap::type,
                                       const rtypes::timestep&,
                                       Ts...>(
          std::forward<decltype(args)>(args)...);
    };
    /*
     * The std::move() is 100% necessary here, because (I think) types in the
     * extra arguments tuple are rvalues, but the values are (probably) lvalues,
     * and if you don't do this you get an error regarding rvalue->lvalue
     * binding.
     */
    return std::apply(lambda, std::move(targs));
  }

  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(constructible_with_mode_and_extra_args<
                               typename TCollectorWrap::type>::value)>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath,
                   const rtypes::timestep& interval,
                   rmetrics::output_mode mode) const {
    m_agg->collector_preregister(scoped_name, mode);
    auto targs =
        std::tuple_cat(std::make_tuple(scoped_name, fpath, interval, mode),
                       mc_extra_args);
    auto lambda = [&](auto&&... args) {
      return m_agg->collector_register<typename TCollectorWrap::type,
                                       const rtypes::timestep&,
                                       rmetrics::output_mode,
                                       Ts...>(
          std::forward<decltype(args)>(args)...);
    };
    /*
     * The std::move() is 100% necessary here, because (I think) types in the
     * extra arguments tuple are rvalues, but the values are (probably) lvalues,
     * and if you don't do this you get an error regarding rvalue->lvalue
     * binding.
     */
    return std::apply(lambda, std::move(targs));
  }

  /**
   * \brief Figure out:
   *
   * - If the selected output mode is valid for the specified collector.
   * - The appropriate filename output stem for the collector if the output
   *   mode is OK.
   *
   * \return (output filepath stem, output interval, output mode) for the
   * collector or empty if the collector fails any pre-initialization checks.
   */
  boost::optional<pre_init_ret_type> collector_pre_initialize(
      const std::string& xml_name,
      rmetrics::output_mode allowed) const {
    auto append_it = mc_config->append.enabled.find(xml_name);
    auto truncate_it = mc_config->truncate.enabled.find(xml_name);
    auto create_it = mc_config->create.enabled.find(xml_name);
    uint sum =
        static_cast<uint>(append_it != mc_config->append.enabled.end()) +
        static_cast<uint>(truncate_it != mc_config->truncate.enabled.end()) +
        static_cast<uint>(create_it != mc_config->create.enabled.end());
    ER_ASSERT(
        sum <= 1,
        "Collector '%s' present in more than 1 collector group in XML file",
        xml_name.c_str());
    if (append_it != mc_config->append.enabled.end()) {
      ER_ASSERT(allowed & rmetrics::output_mode::ekAPPEND,
                "Output mode %d for collector '%s' does not contain ekAPPEND",
                rcppsw::as_underlying(allowed),
                xml_name.c_str());
      auto ret = pre_init_ret_type{m_agg->metrics_path() / append_it->second,
                                   mc_config->append.output_interval,
                                   rmetrics::output_mode::ekAPPEND};
      return boost::make_optional(ret);
    } else if (truncate_it != mc_config->truncate.enabled.end()) {
      ER_ASSERT(allowed & rmetrics::output_mode::ekTRUNCATE,
                "Output mode %d for collector '%s' does not contain ekTRUNCATE",
                rcppsw::as_underlying(allowed),
                xml_name.c_str());
      auto ret = pre_init_ret_type{m_agg->metrics_path() / truncate_it->second,
                                   mc_config->truncate.output_interval,
                                   rmetrics::output_mode::ekTRUNCATE};
      return boost::make_optional(ret);
    } else if (create_it != mc_config->create.enabled.end()) {
      ER_ASSERT(allowed & rmetrics::output_mode::ekCREATE,
                "Output mode %d for collector '%s' does not contain ekCREATE",
                rcppsw::as_underlying(allowed),
                xml_name.c_str());
      /* Give them their own directory to output stuff into for cleanliness */
      auto dirpath = m_agg->metrics_path() / create_it->second;
      fs::create_directories(dirpath);
      auto ret = pre_init_ret_type{dirpath / create_it->second,
                                   mc_config->create.output_interval,
                                   rmetrics::output_mode::ekCREATE};
      return boost::make_optional(ret);
    }
    return boost::optional<pre_init_ret_type>();
  }

  /* clang-format off */
  const std::tuple<Ts...>               mc_extra_args;
  const cmconfig::metrics_config* const mc_config;
  const creatable_set                   mc_create_set;

  base_metrics_aggregator* const        m_agg;
  /* clang-format on */
};

NS_END(detail);

/**
 * \class collector_registerer
 * \ingroup metrics
 *
 * \brief After enabled collectors have been parsed from an XML input file,
 * register the enabled collectors with a \ref base_metrics_aggregator or a
 * class derived from it.
 *
 * This is a separate class from \ref collector_registerer_impl so you don't
 * ALWAYS have to pass it a list of extra arguments for building some finicky
 * metrics collectors.
 */
template <typename TExtraArgsTuple = std::tuple<>>
class collector_registerer
    : public detail::collector_registerer_impl<TExtraArgsTuple> {
 public:
  using detail::collector_registerer_impl<TExtraArgsTuple>::collector_registerer_impl;
  using detail::collector_registerer_impl<TExtraArgsTuple>::operator();
};

NS_END(metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_COLLECTOR_REGISTERER_HPP_ */
