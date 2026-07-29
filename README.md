# romea_core_localisation

## Overview

`romea_core_localisation` is the C++ localisation-domain library used by the ROMEA localisation stack.

It does not provide complete standalone localisation filters by itself. Instead, it provides the localisation-specific components needed to assemble such filters with the generic Kalman or particle filtering machinery from `romea_core_filtering`.

These components include states, predictors, updaters, observations and result extractors used to estimate:

* a robot pose in the world frame;
* the relative pose between two robots;
* the relative position between a robot and a human.

The package is framework-independent C++ code. Middleware-specific nodes, message conversions and runtime configuration are intentionally kept outside this library.

---

## Localisation components

`romea_core_filtering` provides the generic asynchronous filtering layer:

* `FilterBase`, which manages timestamped meta-states and applies prediction and update steps;
* `KalmanFilter` and `ParticleFilter`, which provide the concrete filter storage for Kalman and particle states;
* `FilterPredictorBase`, which defines the prediction interface used between two timestamps;
* Kalman and particle updater base classes and equations, which implement the reusable mathematical parts of the update step.

`romea_core_localisation` provides the localisation-domain components plugged into this filtering layer:

| Component | Purpose |
| --------- | ------- |
| Observations | Typed measurements such as twist, angular speed, position, pose or range, with their uncertainty. |
| Meta-states | Localisation state variables, inputs, covariances and additional data for each localisation problem. |
| Predictors | Motion propagation logic between two timestamps. |
| Proprioceptive updaters | Update motion inputs such as twist, linear speed or angular speed. |
| Exteroceptive updaters | Correct the predicted state from external observations such as position, course, pose or range. |
| Results | Extract estimated poses, twists and uncertainty information from a filter state. |
| Traits | Group the right filter, predictor, updater and result classes for a selected model and filter type. |
| Filter wrapper | Collects localisation updaters, configures proprioceptive observation age limits, initializes the concrete core filter and returns localisation query results. |

Applications assemble a complete filter by selecting a filter type, creating a localisation predictor, adding the updaters provided by this package and calling `initialize()` once the updater set is complete.

---

## Localisation models

| Model | Namespace / prefix | Purpose |
| ----- | ------------------ | ------- |
| Robot-to-world | `R2W*` | Estimate the vehicle pose in a world frame from proprioceptive and exteroceptive observations. |
| Robot-to-robot | `R2R*` | Estimate the relative pose between a follower robot and a leader robot. |
| Robot-to-human | `R2H*` | Estimate the relative position between a robot and a human. |

These model families are independent C++ components. Application or integration packages choose which model to assemble and expose.

---

## Observations

The library defines typed Gaussian observations that can be fused by localisation filters.

| Observation | Content |
| ----------- | ------- |
| `ObservationTwist` | 2D body twist observation. |
| `ObservationLinearSpeed` | Single longitudinal speed observation. |
| `ObservationLinearSpeeds` | Left/right longitudinal speed observations. |
| `ObservationAngularSpeed` | Yaw angular speed observation. |
| `ObservationAttitude` | Roll and pitch attitude observation. |
| `ObservationPosition` | 2D position observation. |
| `ObservationCourse` | Course or heading observation. |
| `ObservationPose` | 2D pose observation. |
| `ObservationRange` | Range observation between two points or transceivers. |

Each observation carries both a value and uncertainty information, either as a standard deviation or covariance depending on its dimension.

---

## Predictors and updaters

The localisation filters are built from:

* predictors, which propagate the state using the current proprioceptive inputs;
* proprioceptive updaters, which update internal motion inputs such as twist, linear speed or angular speed;
* exteroceptive updaters, which correct the state with observations such as position, course, pose or range.

Proprioceptive updaters also declare which filter inputs they feed. The filter wrapper merges these constraints before initialization and forwards the resulting observation age limits to the predictor. If a required proprioceptive input becomes too old, prediction can reset the localisation state instead of continuing with stale motion data.

Exteroceptive observations are allowed to be intermittent. Loss of exteroceptive observations is handled by the dead-reckoning limits tracked by the predictor.

Exteroceptive updaters support trigger modes:

| Trigger | Meaning |
| ------- | ------- |
| `always` | Each valid observation can update the filter. |
| `once` | Only the first valid observation is used, commonly for initialisation. |

Kalman exteroceptive updaters support Mahalanobis-distance based rejection to discard observations that are inconsistent with the current filter state. Particle exteroceptive updaters provide the particle weighting and resampling logic for the corresponding observations.

---

## Available component sets

The package provides Kalman and particle component sets for the currently implemented localisation models:

| Model | Kalman components | Particle components | Traits |
| ----- | ----------------- | ------------------- | ------ |
| Robot-to-world | `robot_to_world/kalman/*` | `robot_to_world/particle/*` | `R2WTraits<KALMAN>`, `R2WTraits<PARTICLE>` |
| Robot-to-robot | `robot_to_robot/kalman/*` | `robot_to_robot/particle/*` | `R2RTraits<KALMAN>`, `R2RTraits<PARTICLE>` |
| Robot-to-human | `robot_to_human/kalman/*` | Not currently provided in this package. | `R2HTraits<KALMAN>` |

For example, a robot-to-world Kalman localisation filter is assembled from the following component families:

| Component | Main class or examples | Role |
| --------- | ---------------------- | ---- |
| Filter | `romea::core::KalmanFilter<R2WKFMetaState, FSMState, Duration>` | Manages timestamped states asynchronously and applies prediction and update steps. |
| Meta-state | `R2WKFMetaState` | Stores the vehicle pose, motion inputs and covariance information. |
| FSM state | `FSMState` | Tracks the filter status associated with each stored state: `INIT`, `RUNNING`, `RESET`, `STALE` or `ABORTED`. |
| Predictor | `R2WKFPredictor` | Propagates the vehicle state from motion inputs and the vehicle kinematic model. |
| Exteroceptive updaters | `R2WKFUpdaterPosition`, `R2WKFUpdaterCourse`, `R2WKFUpdaterPose`, `R2WKFUpdaterRange` | Correct the predicted state with external observations. |
| Proprioceptive updaters | `UpdaterTwist`, `UpdaterLinearSpeed`, `UpdaterLinearSpeeds`, `UpdaterAngularSpeed`, `R2WUpdaterAttitude` | Update the motion inputs and attitude data used by the predictor. |
| Results | `R2WResults` | Stores the estimated pose, twist and covariance information converted from the current filter state. |

The corresponding traits can be used to select the right component set:

```cpp
using Traits = romea::core::localisation::R2WTraits<romea::core::KALMAN>;
using LocalisationFilter =
  romea::core::localisation::Filter<romea::core::KALMAN, Traits>;

const romea::core::localisation::DeadReckoningLimits dead_reckoning_limits(
  romea::core::durationFromSecond(maximal_dead_reckoning_elapsed_time),
  maximal_dead_reckoning_travelled_distance);

auto predictor = std::make_unique<Traits::Predictor>(dead_reckoning_limits);
auto filter = std::make_unique<LocalisationFilter>(state_pool_size, std::move(predictor));

auto position_updater = std::make_unique<Traits::UpdaterPosition>(
  "position_updater",
  minimal_rate,
  trigger_mode,
  mahalanobis_distance_rejection_threshold);

auto twist_updater = std::make_unique<Traits::UpdaterTwist>(
  "twist_updater",
  minimal_rate);

auto twist_callback = filter->add_updater(std::move(twist_updater));
auto position_callback = filter->add_updater(std::move(position_updater));

filter->initialize();

if (twist_callback) {
  (*twist_callback)(twist_observation_time, twist_observation);
}

if (position_callback) {
  (*position_callback)(position_observation_time, position_observation);
}

const auto query = filter->get_results(query_time);
if (query) {
  const auto & current_pose = query.results->robot_pose;
  auto current_status = query.fsm_state;
}
```

`get_results()` returns a `FilterQuery<Results>`. Results are present only when the underlying filter query is available and the localisation FSM state is `RUNNING`; the FSM state is still returned so callers can publish status and make diagnostics decisions.

This example only shows the assembly principle. Real applications usually build observations inside sensor callbacks, add several proprioceptive and exteroceptive updaters, call `initialize()` after all updaters have been added, and use the returned query state for diagnostics and publication decisions.

---

## Related packages

| Package | Role |
| ------- | ---- |
| `romea_core_filtering` | Generic Kalman and particle filtering layer. |
| `romea_core_localisation_gps` | GPS-to-localisation observation utilities. |
| `romea_core_localisation_imu` | IMU-to-localisation observation utilities. |
| `romea_core_localisation_rtls` | RTLS-to-localisation observation utilities. |

---

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This library was developed by **Jean Laneurit** with scientific contributions from **Christophe Debain**, **Roland Chapuis** and **Romuald Aufrere**, in the context of several research projects including Baudet ROB, Baudet Rob 2 and Adap2E.
