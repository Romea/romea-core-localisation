// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_CORE_LOCALISATION__FILTER_HPP_
#define ROMEA_CORE_LOCALISATION__FILTER_HPP_

// std
#include <cassert>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

// romea
#include "romea_core_common/diagnostic/DiagnosticReport.hpp"
#include "romea_core_filtering/filter/filter_base.hpp"
#include "romea_core_filtering/filter/type.hpp"
#include "romea_core_localisation/dead_reckoning_tracking.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_tracking.hpp"
#include "romea_core_localisation/updater_exteroceptive.hpp"
#include "romea_core_localisation/updater_proprioceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<class Results>
struct FilterQuery
{
  FSMState fsm_state;
  std::optional<Results> results;

  bool has_results() const { return results.has_value(); }

  explicit operator bool() const { return has_results(); }
};

template<class Traits>
class FilterBase
{
private:
  class UpdaterHandleBase
  {
  public:
    virtual ~UpdaterHandleBase() = default;

    virtual bool heart_beat_callback(const Duration & duration) = 0;

    virtual DiagnosticReport get_report() = 0;
  };

  template<class Updater>
  class UpdaterHandle : public UpdaterHandleBase
  {
  public:
    explicit UpdaterHandle(std::unique_ptr<Updater> updater) : updater_(std::move(updater)) {}

    Updater & updater() { return *updater_; }

    bool heart_beat_callback(const Duration & duration) override
    {
      return updater_->heart_beat_callback(duration);
    }

    DiagnosticReport get_report() override { return updater_->get_report(); }

  private:
    std::unique_ptr<Updater> updater_;
  };

public:
  using Filter = typename Traits::Filter;
  using MetaState = typename Traits::MetaState;
  using Predictor = typename Traits::Predictor;
  using Results = typename Traits::Results;
  using MetaStateToResults = typename Traits::MetaStateToResults;
  using ObservationAgeLimits = typename Traits::ObservationAgeLimits;
  using LogCallback = std::function<void(const std::string &)>;
  template<class Updater>
  using UpdateCallback =
    std::function<void(const Duration &, const typename Updater::Observation &)>;

protected:
  FilterBase(
    std::unique_ptr<Filter> filter,
    std::unique_ptr<Predictor> predictor,
    std::unique_ptr<MetaState> current_meta_state,
    std::unique_ptr<MetaStateToResults> meta_state_to_results)
  : filter_(std::move(filter)),
    predictor_(std::move(predictor)),
    current_meta_state_(std::move(current_meta_state)),
    meta_state_to_results_(std::move(meta_state_to_results)),
    proprioceptive_updater_handles_(),
    exteroceptive_updater_handles_(),
    event_log_callback_(),
    get_results_log_callback_(),
    is_initialized_(false)
  {
  }

public:
  void register_event_log_callback(LogCallback callback)
  {
    event_log_callback_ = std::move(callback);
  }

  void register_get_results_log_callback(LogCallback callback)
  {
    get_results_log_callback_ = std::move(callback);
  }

  bool initialize()
  {
    if (is_initialized_) {
      log_event_("filter is already initialized");
      return true;
    }

    if (!predictor_) {
      log_event_("cannot initialize filter because predictor is not configured");
      return false;
    }

    filter_->register_predictor(std::move(predictor_));
    is_initialized_ = true;

    log_event_("filter initialized");

    return true;
  }

  void reset() { filter_->reset(); }

  FilterQuery<Results> get_results(const Duration & duration)
  {
    using QueryStatus = typename ::romea::core::FilterStateQueryResult<FSMState>::Status;

    if (!is_initialized_) {
      log_get_results_("cannot get results because filter is not initialized");
      return {FSMState::INIT, std::nullopt};
    }

    const auto filter_query = filter_->get_state(duration, current_meta_state_.get());
    FilterQuery<Results> localisation_query{filter_query.fsm_state, std::nullopt};

    switch (filter_query.status) {
      case QueryStatus::AVAILABLE:
        if (filter_query.fsm_state != FSMState::RUNNING) {
          log_get_results_("cannot get results because filter is not running");
          return localisation_query;
        }
        localisation_query.results = meta_state_to_results_->convert(*current_meta_state_);
        return localisation_query;
      case QueryStatus::EMPTY:
        log_get_results_("cannot get results because filter has no state yet");
        return localisation_query;
      case QueryStatus::TOO_OLD:
        log_get_results_("cannot get results because requested state is too old");
        return localisation_query;
      case QueryStatus::TOO_FAR:
        log_get_results_("cannot get results because requested state is too far");
        return localisation_query;
      default:
        assert(false);
        return localisation_query;
    }
  }

  DiagnosticReport make_diagnostic_report(const Duration & duration, const FSMState & fsm_state)
  {
    DiagnosticReport report;

    for (auto & updater_handle : proprioceptive_updater_handles_) {
      updater_handle->heart_beat_callback(duration);
      report += updater_handle->get_report();
    }

    for (auto & updater_handle : exteroceptive_updater_handles_) {
      updater_handle->heart_beat_callback(duration);
      auto exteroceptive_report = updater_handle->get_report();
      downgrade_exteroceptive_diagnostics_(exteroceptive_report, fsm_state);
      report += exteroceptive_report;
    }

    return report;
  }

  template<class Updater>
  std::optional<UpdateCallback<Updater>> add_updater(std::unique_ptr<Updater> updater)
  {
    if constexpr (std::is_base_of_v<UpdaterProprioceptive, Updater>) {
      return add_proprioceptive_updater(std::move(updater));
    } else if constexpr (std::is_base_of_v<UpdaterExteroceptive, Updater>) {
      return add_exteroceptive_updater(std::move(updater));
    } else {
      static_assert(
        std::is_base_of_v<UpdaterProprioceptive, Updater> ||
          std::is_base_of_v<UpdaterExteroceptive, Updater>,
        "Updater must inherit from UpdaterProprioceptive or UpdaterExteroceptive");
      }
  }

private:
  template<class Updater>
  std::optional<UpdateCallback<Updater>> add_proprioceptive_updater(
    std::unique_ptr<Updater> updater)
  {
    if (is_initialized_) {
      log_event_("cannot add proprioceptive updater because filter is already initialized");
      return std::nullopt;
    }

    ObservationAgeLimits observation_age_limits;
    Updater::update_observation_age_limits(observation_age_limits, updater->get_minimal_rate());
    predictor_->update_observation_age_limits(observation_age_limits);

    auto * updater_ptr = updater.get();

    proprioceptive_updater_handles_.push_back(
      std::make_unique<UpdaterHandle<Updater>>(std::move(updater)));

    return make_update_callback_(updater_ptr);
  }

  template<class Updater>
  std::optional<UpdateCallback<Updater>> add_exteroceptive_updater(std::unique_ptr<Updater> updater)
  {
    if (is_initialized_) {
      log_event_("cannot add exteroceptive updater because filter is already initialized");
      return std::nullopt;
    }

    auto * updater_ptr = updater.get();

    exteroceptive_updater_handles_.push_back(
      std::make_unique<UpdaterHandle<Updater>>(std::move(updater)));

    return make_update_callback_(updater_ptr);
  }

  template<class Updater>
  UpdateCallback<Updater> make_update_callback_(Updater * updater_ptr)
  {
    return UpdateCallback<Updater>(
      [this, updater_ptr](
        const Duration & duration, const typename Updater::Observation & observation) {
        filter_->process(
          duration,
          [updater_ptr, observation](
            const Duration & update_duration, FSMState & fsm_state, MetaState & meta_state) {
            updater_ptr->update(update_duration, observation, fsm_state, meta_state);
          });
      });
  }

  void log_event_(const std::string & message)
  {
    if (event_log_callback_) {
      event_log_callback_(message);
    }
  }

  void log_get_results_(const std::string & message)
  {
    if (get_results_log_callback_) {
      get_results_log_callback_(message);
    }
  }

  void downgrade_exteroceptive_diagnostics_(
    DiagnosticReport & report, const FSMState & fsm_state) const
  {
    if (fsm_state != FSMState::RUNNING) {
      return;
    }

    for (auto & diagnostic : report.diagnostics) {
      if (
        diagnostic.status != DiagnosticStatus::OK && diagnostic.status != DiagnosticStatus::WARN) {
        diagnostic.status = DiagnosticStatus::WARN;
      }
    }
  }

protected:
  std::unique_ptr<Filter> filter_;
  std::unique_ptr<Predictor> predictor_;
  std::unique_ptr<MetaState> current_meta_state_;
  std::unique_ptr<MetaStateToResults> meta_state_to_results_;
  std::list<std::unique_ptr<UpdaterHandleBase>> proprioceptive_updater_handles_;
  std::list<std::unique_ptr<UpdaterHandleBase>> exteroceptive_updater_handles_;
  LogCallback event_log_callback_;
  LogCallback get_results_log_callback_;
  bool is_initialized_;
};

template<FilterType FilterType_, class Traits>
class Filter;

template<class Traits>
class Filter<KALMAN, Traits> : public FilterBase<Traits>
{
private:
  using Base = FilterBase<Traits>;

public:
  explicit Filter(
    const std::size_t & state_pool_size, std::unique_ptr<typename Base::Predictor> predictor)
  : Base(
      std::make_unique<typename Base::Filter>(state_pool_size),
      std::move(predictor),
      std::make_unique<typename Base::MetaState>(),
      std::make_unique<typename Base::MetaStateToResults>())
  {
  }
};

template<class Traits>
class Filter<PARTICLE, Traits> : public FilterBase<Traits>
{
private:
  using Base = FilterBase<Traits>;

public:
  Filter(
    const std::size_t & state_pool_size,
    const std::size_t & number_of_particles,
    std::unique_ptr<typename Base::Predictor> predictor)
  : Base(
      std::make_unique<typename Base::Filter>(state_pool_size, number_of_particles),
      std::move(predictor),
      std::make_unique<typename Base::MetaState>(number_of_particles),
      std::make_unique<typename Base::MetaStateToResults>(number_of_particles))
  {
  }
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__FILTER_HPP_
