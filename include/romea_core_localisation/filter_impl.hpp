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

#ifndef ROMEA_CORE_LOCALISATION__FILTER_IMPL_HPP_
#define ROMEA_CORE_LOCALISATION__FILTER_IMPL_HPP_

// std
#include <cassert>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

// romea
#include "romea_core_common/diagnostic/DiagnosticReport.hpp"
#include "romea_core_filtering/filter/filter_base.hpp"
#include "romea_core_filtering/filter/type.hpp"
#include "romea_core_localisation/dead_reckoning_tracking.hpp"
#include "romea_core_localisation/filter_storage.hpp"
#include "romea_core_localisation/fsm_state.hpp"
#include "romea_core_localisation/observation_tracking.hpp"
#include "romea_core_localisation/updater_proprioceptive.hpp"

namespace romea
{
namespace core
{
namespace localisation
{

template<FilterType FilterType_, class Traits>
class Filter;

template<class Results>
struct FilterQuery
{
  FSMState fsm_state;
  std::optional<Results> results;

  bool has_results() const { return results.has_value(); }

  explicit operator bool() const { return has_results(); }
};

template<class Traits>
class FilterImpl
{
private:
  using Storage = FilterStorage<Traits>;

public:
  using Filter = typename Storage::Filter;
  using MetaState = typename Storage::MetaState;
  using Predictor = typename Storage::Predictor;
  using Results = typename Storage::Results;
  using MetaStateToResults = typename Storage::MetaStateToResults;
  using ObservationAgeLimits = typename Traits::ObservationAgeLimits;
  using LogCallback = std::function<void(const std::string &)>;
  using LoggerMap = std::map<std::string, std::shared_ptr<Logger>>;

  template<class Updater>
  using UpdateCallback = typename Storage::template UpdateCallback<Updater>;
  template<class Updater>
  using UpdaterHandle = typename Storage::template UpdaterHandle<Updater>;

public:
  FilterImpl(
    std::unique_ptr<Filter> filter,
    std::unique_ptr<Predictor> predictor,
    std::unique_ptr<MetaState> current_meta_state,
    std::unique_ptr<MetaStateToResults> meta_state_to_results)
  : storage_(), event_log_callback_(), get_results_log_callback_(), is_initialized_(false)
  {
    assert(filter && "cannot create filter impl with null core filter");
    assert(predictor && "cannot create filter impl with null predictor");
    assert(current_meta_state && "cannot create filter impl with null current meta state");
    assert(meta_state_to_results && "cannot create filter impl with null meta state to results");

    storage_.filter = std::move(filter);
    storage_.predictor = std::move(predictor);
    storage_.current_meta_state = std::move(current_meta_state);
    storage_.meta_state_to_results = std::move(meta_state_to_results);
  }

  virtual ~FilterImpl() = default;

  bool initialize(
    LogCallback event_log_callback,
    LogCallback get_results_log_callback,
    FSMEventCallback fsm_event_callback = nullptr,
    const LoggerMap & loggers = LoggerMap())
  {
    if (is_initialized_) {
      log_event_("filter is already initialized");
      return true;
    }

    event_log_callback_ = std::move(event_log_callback);
    get_results_log_callback_ = std::move(get_results_log_callback);

    if (fsm_event_callback) {
      register_fsm_callback(storage_, fsm_event_callback);
    }

    if (!loggers.empty()) {
      register_topic_loggers(storage_, loggers);
    }

    storage_.filter->register_predictor(std::move(storage_.predictor));
    is_initialized_ = true;

    log_event_("filter initialized");

    return true;
  }

public:
  void reset()
  {
    if (storage_.filter) {
      storage_.filter->reset();
    }
  }

  FilterQuery<Results> get_results(const Duration & duration)
  {
    using QueryStatus = typename ::romea::core::FilterStateQueryResult<FSMState>::Status;

    if (!is_initialized_) {
      log_get_results_("cannot get results because filter is not initialized");
      return {FSMState::INIT, std::nullopt};
    }

    const auto filter_query =
      storage_.filter->get_state(duration, storage_.current_meta_state.get());
    FilterQuery<Results> localisation_query{filter_query.fsm_state, std::nullopt};

    switch (filter_query.status) {
      case QueryStatus::AVAILABLE:
        if (filter_query.fsm_state != FSMState::RUNNING) {
          log_get_results_("cannot get results because filter is not in running state");
          return localisation_query;
        }
        localisation_query.results =
          storage_.meta_state_to_results->convert(*storage_.current_meta_state);
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
    auto report =
      localisation::make_diagnostic_report(duration, storage_.proprioceptive_updater_handles);
    auto exteroceptive_report =
      localisation::make_diagnostic_report(duration, storage_.exteroceptive_updater_handles);

    downgrade_exteroceptive_diagnostics_(exteroceptive_report, fsm_state);
    report += exteroceptive_report;

    return report;
  }

  template<class Updater>
  std::optional<UpdateCallback<Updater>> add_proprioceptive_updater(
    const std::string & updater_name, std::unique_ptr<Updater> updater)
  {
    if (is_initialized_) {
      log_event_("cannot add proprioceptive updater because filter is already initialized");
      return std::nullopt;
    }

    if (storage_.proprioceptive_updater_handles.count(updater_name) > 0) {
      log_event_("cannot add proprioceptive updater because name already exists: " + updater_name);
      return std::nullopt;
    }

    ObservationAgeLimits observation_age_limits;
    Updater::update_observation_age_limits(observation_age_limits, updater->get_minimal_rate());
    storage_.predictor->update_observation_age_limits(observation_age_limits);

    auto * updater_ptr = updater.get();

    storage_.proprioceptive_updater_handles.emplace(
      updater_name, std::make_unique<UpdaterHandle<Updater>>(std::move(updater)));

    return make_update_callback_(updater_ptr);
  }

  template<class Updater>
  std::optional<UpdateCallback<Updater>> add_exteroceptive_updater(
    const std::string & updater_name, std::unique_ptr<Updater> updater)
  {
    if (is_initialized_) {
      log_event_("cannot add exteroceptive updater because filter is already initialized");
      return std::nullopt;
    }

    if (storage_.exteroceptive_updater_handles.count(updater_name) > 0) {
      log_event_("cannot add exteroceptive updater because name already exists: " + updater_name);
      return std::nullopt;
    }

    auto * updater_ptr = updater.get();

    storage_.exteroceptive_updater_handles.emplace(
      updater_name, std::make_unique<UpdaterHandle<Updater>>(std::move(updater)));

    return make_update_callback_(updater_ptr);
  }

private:
  template<class Updater>
  UpdateCallback<Updater> make_update_callback_(Updater * updater_ptr)
  {
    return UpdateCallback<Updater>(
      [this, updater_ptr](
        const Duration & duration, const typename Updater::Observation & observation) {
        assert(is_initialized_ && "cannot update filter before initialization");
        assert(storage_.filter && "cannot update filter without core filter");

        storage_.filter->process(
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

private:
  Storage storage_;
  LogCallback event_log_callback_;
  LogCallback get_results_log_callback_;
  bool is_initialized_;
};

}  // namespace localisation
}  // namespace core
}  // namespace romea

#endif  // ROMEA_CORE_LOCALISATION__FILTER_IMPL_HPP_
