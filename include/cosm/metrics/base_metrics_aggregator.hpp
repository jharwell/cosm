/**
 * \file base_metrics_aggregator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_METRICS_BASE_METRICS_AGGREGATOR_HPP_
#define INCLUDE_COSM_METRICS_BASE_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <map>
#include <string>
#include <typeindex>
#include <utility>
#include <filesystem>

#include "rcppsw/er/client.hpp"
#include "rcppsw/metrics/collector_group.hpp"

#include "cosm/metrics/config/metrics_config.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller {
class base_controller2D;
class base_controllerQ3D;
} /* namespace cosm::controller */

NS_START(cosm, metrics);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_metrics_aggregator
 * \ingroup metrics
 *
 * \brief Base class for aggregating collection of metrics from various
 * sources across all possible collector output modes.
 */
class base_metrics_aggregator : public rer::client<base_metrics_aggregator> {
 public:
  base_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                          const std::string& output_root);
  ~base_metrics_aggregator(void) override = default;

  const fs::path& metrics_path(void) const { return m_metrics_path; }

  /**
   * \brief Collect metrics from a 2D block right before it is dropped in the
   * nest.
   */
  void collect_from_block(const crepr::base_block3D* block);

  /**
   * \brief Collect metrics from a 3D block right before it is dropped in the
   * nest.
   */
  void collect_from_block(const crepr::base_block2D* block);

  /**
   * \brief Collect metrics from 2D controllers. Currently this includes:
   *
   * - \ref spatial::dist2D_metrics
   */
  void collect_from_controller(const controller::base_controller2D* controller);

  /**
   * \brief Collect metrics from Q3D controllers. Currently this includes:
   *
   * - \ref spatial::dist3D_metrics
   */
  void collect_from_controller(const controller::base_controllerQ3D* controller);

  /**
   * \brief To be called before \ref collector_register(), in order to correctly
   * set up the collector map for the collector with the specified scoped
   * name.
   *
   * If you forget to do this you will get a segfault.
   */
  void collector_preregister(const std::string& scoped_name,
                             rmetrics::output_mode mode) {
    if (rmetrics::output_mode::ekAPPEND == mode) {
      m_collector_map[scoped_name] = &m_append;
    } else if (rmetrics::output_mode::ekTRUNCATE == mode) {
      m_collector_map[scoped_name] = &m_truncate;
    } else if (rmetrics::output_mode::ekCREATE == mode) {
      m_collector_map[scoped_name] = &m_create;
    }
  }

  /**
   * \brief Decorator around \ref collector_group::collector_register().
   */
  template <typename TCollectorType, typename... Args>
  bool collector_register(const std::string& scoped_name,
                          const std::string& fpath,
                          Args&&... args) {
    return m_collector_map[scoped_name]->collector_register<TCollectorType>(
        scoped_name, fpath, std::forward<Args>(args)...);
  }

  void reset_all(void) {
    m_create.reset_all();
    m_append.reset_all();
    m_truncate.reset_all();
  }

  /**
   * \brief Decorator around \ref collector_group::collect().
   */
  template <typename T>
  void collect(const std::string& scoped_name, const T& collectee) {
    auto it = m_collector_map.find(scoped_name);
    if (it != m_collector_map.end()) {
      it->second->collect(scoped_name, collectee);
    }
  } /* collect() */

  /**
   * \brief Decorator around \ref collector_group::collect_if().
   */
  template <typename T>
  void collect_if(const std::string& scoped_name,
                  const T& collectee,
                  const std::function<bool(const rmetrics::base_metrics&)>& pred) {
    auto it = m_collector_map.find(scoped_name);
    if (it != m_collector_map.end()) {
      it->second->collect_if(scoped_name, collectee, pred);
    }
  } /* collect() */

  /**
   * \brief Decorator around \ref collector_group::collector_remove().
   */
  bool collector_remove(const std::string& scoped_name) {
    auto it = m_collector_map.find(scoped_name);
    if (it != m_collector_map.end()) {
      return it->second->collector_remove(scoped_name);
    }
    return false;
  }

  /**
   * \brief Decorator around \ref collector_group::get().
   */
  template <typename T>
  const T* get(const std::string& key) {
    return m_collector_map[key]->get<T>(key);
  }

  bool metrics_write(rmetrics::output_mode mode) {
    if (rmetrics::output_mode::ekAPPEND == mode) {
      return m_append.metrics_write_all();
    } else if (rmetrics::output_mode::ekTRUNCATE == mode) {
      return m_truncate.metrics_write_all();
    } else if (rmetrics::output_mode::ekCREATE == mode) {
      return m_create.metrics_write_all();
    }
    return false;
  }

  /**
   * \brief Decorator around \ref collector_group::timestep_inc_all().
   */
  void timestep_inc_all(void) {
    m_append.timestep_inc_all();
    m_truncate.timestep_inc_all();
    m_create.timestep_inc_all();
  }

  /**
   * \brief Decorator around \ref collector_group::interval_reset_all().
   */
  void interval_reset_all(void) {
    m_append.interval_reset_all();
    m_truncate.interval_reset_all();
    m_create.interval_reset_all();
  }

  /**
   * \brief Decorator around \ref collector_group::finalize_all().
   */
  void finalize_all(void) {
    m_append.finalize_all();
    m_truncate.finalize_all();
    m_create.finalize_all();
  }

 protected:
  /**
   * \brief Register metrics collectors that require the arena dimensions to
   * construct for robots in a structure 2D environment.
   */
  void register_with_arena_dims2D(const cmconfig::metrics_config* mconfig,
                                  const rmath::vector2z& dims);

  /**
   * \brief Register metrics collectors that require the arena dimensions AND
   * the maximum height robots can access within the arena.
   */
  void register_with_arena_dims3D(
      const cmconfig::metrics_config* mconfig,
      const rmath::vector3z& dims);

 private:
  /**
   * \brief Maps the scoped name of the collector to the \ref collector_group it
   * belongs in.
   */
  using collector_map_type = std::map<std::string, rmetrics::collector_group*>;

  /**
   * \brief Register metrics collectors that do not require extra arguments.
   */
  void register_standard(const cmconfig::metrics_config* mconfig);

  /* clang-format off */
  fs::path                  m_metrics_path;
  collector_map_type        m_collector_map{};
  rmetrics::collector_group m_append{};
  rmetrics::collector_group m_truncate{};
  rmetrics::collector_group m_create{};
  /* clang-format on */
};

NS_END(metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_BASE_METRICS_AGGREGATOR_HPP_ */
