#include "optimizer.h"
#include "esphome/components/ecodan/ecodan.h"

// Definition of tolerance to avoid floating point comparison issues
// and force sending even if the deviation is small but existing.
#define FLOW_TEMP_TOLERANCE 0.1f

using std::isnan;

namespace esphome {
namespace optimizer {

using namespace esphome::ecodan;

Optimizer::Optimizer(OptimizerState state) : state_(state) {
    this->total_bias_sensor_ = state.total_bias_sensor;
    this->target_delta_t_sensor_ = state.target_delta_t_sensor;
    this->cold_factor_sensor_ = state.cold_factor_sensor;

    auto update_if_changed = [](float &storage, float new_val, auto callback) {
        if (std::isnan(new_val)) return;
        if (std::isnan(storage) || std::abs(storage - new_val) > 0.01f) {
            // Store new value first then invoke callback
            auto previous = storage;
            storage = new_val;
            callback(new_val, previous);
        }
    };

    if (this->state_.hp_feed_temp != nullptr) {
        this->state_.hp_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_hp_feed_temp_, x, [this](float new_v, float old_v) {
                this->on_feed_temp_change(new_v, OptimizerZone::SINGLE);
            });
        });
    }

    if (this->state_.z1_feed_temp != nullptr) {
        this->state_.z1_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_z1_feed_temp_, x, [this](float new_v, float old_v) {
                this->on_feed_temp_change(new_v, OptimizerZone::ZONE_1);
            });
        });
    }

    if (this->state_.z2_feed_temp != nullptr) {
        this->state_.z2_feed_temp->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_z2_feed_temp_, x, [this](float new_v, float old_v) {
                this->on_feed_temp_change(new_v, OptimizerZone::ZONE_2);
            });
        });
    }

    if (this->state_.operation_mode != nullptr) {
        this->state_.operation_mode->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_operation_mode_, x, [this](float new_v, float old_v) {
                this->on_operation_mode_change(static_cast<uint8_t>(new_v), static_cast<uint8_t>(old_v));
            });
        });
    }

    if (this->state_.status_compressor != nullptr) {
        this->state_.status_compressor->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_compressor_status_, x, [this](float new_v, float old_v) {
                this->on_compressor_state_change(static_cast<bool>(new_v), static_cast<bool>(old_v));
            });
        });
    }
    if (this->state_.status_defrost != nullptr) {
        this->state_.status_defrost->add_on_state_callback([this, update_if_changed](float x) {
            update_if_changed(this->last_defrost_status_, x, [this](float new_v, float old_v) {
                this->on_defrost_state_change(static_cast<bool>(new_v), static_cast<bool>(old_v));
            });
        });
    }

    // Initialize preference storage for integral_error_z1_
    this->integral_pref_ = esphome::global_preferences->make_preference<float>(esphome::fnv1_hash("optimizer.integral_error_z1"));
    float saved_integral = 0.0f;
    if (this->integral_pref_.load<float>(&saved_integral)) {
        this->integral_error_z1_ = saved_integral;
        this->last_saved_integral_ = saved_integral;
        ESP_LOGD(OPTIMIZER_TAG, "Loaded saved integral_error_z1_: %.3f", saved_integral);
    } else {
        ESP_LOGD(OPTIMIZER_TAG, "No saved integral_error_z1_, using default %.3f", this->integral_error_z1_);
    }
}

void Optimizer::process_adaptive_zone_(
    std::size_t i,
    const ecodan::Status &status,
    float cold_factor,
    float min_delta_cold_limit,
    float base_min_delta_t,
    float max_delta_t,
    float max_error_range,
    float actual_outside_temp,
    float zone_max_flow_temp,
    float zone_min_flow_temp,
    float &out_flow_heat,
    float &out_flow_cool) 
{
    auto zone = (i == 0) ? esphome::ecodan::Zone::ZONE_1 : esphome::ecodan::Zone::ZONE_2;
    auto heating_type_index = this->state_.heating_system_type->active_index().value_or(0);

    bool is_heating_mode = status.is_auto_adaptive_heating(zone);
    bool is_heating_active = status.Operation == esphome::ecodan::Status::OperationMode::HEAT_ON;
    bool is_cooling_mode = status.has_cooling() && status.is_auto_adaptive_cooling(zone);
    bool is_cooling_active = status.Operation == esphome::ecodan::Status::OperationMode::COOL_ON;

    if (is_heating_active && status.has_2zones()) {
        auto multizone_status = status.MultiZoneStatus;
        if (i == 0 && (multizone_status == 0 || multizone_status == 3)) {
            is_heating_active = false;
        } else if (i == 1 && (multizone_status == 0 || multizone_status == 2)) {
            is_heating_active = false;
        }

        // For z2 with z1/z2 circulation pump and mixing tank, demand translates into pump being active
        if (status.has_independent_z2() && (status.WaterPump2Active || status.WaterPump3Active))
            is_heating_active = true;
    }

    float setpoint_bias = this->state_.auto_adaptive_setpoint_bias->state;
    if (isnan(setpoint_bias))
        setpoint_bias = 0.0f;

    float room_temp = (i == 0) ? status.Zone1RoomTemperature : status.Zone2RoomTemperature;
    float room_target_temp = (i == 0) ? status.Zone1SetTemperature : status.Zone2SetTemperature;
    float requested_flow_temp = (i == 0) ? status.Zone1FlowTemperatureSetPoint : status.Zone2FlowTemperatureSetPoint;
    float actual_flow_temp = status.has_independent_z2() ? ((i == 0) ? status.Z1FeedTemperature : status.Z2FeedTemperature) : status.HpFeedTemperature;
    float actual_return_temp = status.has_independent_z2() ? ((i == 0) ? status.Z1ReturnTemperature : status.Z2ReturnTemperature) : status.HpReturnTemperature;

    if (!is_heating_mode && !is_cooling_mode) {
        // If zone 1 is disabled, reset the integrator
        if (i == 0) {
            this->integral_error_z1_ = 0.0f;
            // Persist reset value
            if (!std::isnan(this->last_saved_integral_)) {
                this->integral_pref_.save<float>(&this->integral_error_z1_);
                this->last_saved_integral_ = this->integral_error_z1_;
            }
        }
        return;
    }

    if (this->state_.temperature_feedback_source->active_index().value_or(0) == 1) {
        room_temp = (i == 0) ? this->state_.temperature_feedback_z1->state : this->state_.temperature_feedback_z2->state;
    }
    if (isnan(room_temp) || isnan(room_target_temp) || isnan(requested_flow_temp) || isnan(actual_flow_temp))
        return;

    // ==========================================
    // PID LOGIC (Zone 1 only) - 5 Minute Cycle
    // ==========================================
    float total_bias = setpoint_bias;

    if (i == 0) {
        float raw_error = room_target_temp - room_temp;

        // --- 1. DERIVATIVE TERM CALCULATION (D) ---
        // With a 5-minute cycle, we analyze the variation since the last run.

        uint32_t now = millis();

        // Ensure this isn't the first run (last_time != 0) and we have valid data
        if (this->last_derivative_time_ != 0 && !std::isnan(this->last_room_temp_z1_)) {
            
            float dt = (now - this->last_derivative_time_) / 1000.0f; // Elapsed time in seconds (expected ~300s)

            // Safety check: If dt is too small (< 60s) or too large (> 15 min, e.g., power outage),
            // we skip the D calculation for this cycle to avoid sudden jumps.
            if (dt > 60.0f && dt < 900.0f) {
                float temp_diff = room_temp - this->last_room_temp_z1_;
                float slope = temp_diff / dt; // °C per second

                // GAIN CALCULATION
                // If Temp rises by 0.2°C in 5 min: Slope = 0.2 / 300 = 0.00066
                // To correct flow temp by -1°C, we need a Gain of approx 1500.
                const float DERIVATIVE_GAIN = 200.0f;

                this->derivative_term_ = slope * DERIVATIVE_GAIN;

                ESP_LOGD(OPTIMIZER_TAG, "Z1 PID-D (5min cycle): Diff=%.2f°C, dt=%.0fs, Slope=%.6f, D-Term=%.3f",
                         temp_diff, dt, slope, this->derivative_term_);
            } else {
                ESP_LOGW(OPTIMIZER_TAG, "Z1 PID-D: Skipped due to irregular time delta: %.1fs", dt);
            }
        } else {
            // First cycle: no derivative possible
            this->derivative_term_ = 0.0f;
        }

        // Update values for the next 5-minute cycle
        this->last_room_temp_z1_ = room_temp;
        this->last_derivative_time_ = now;


        // --- 2. INTEGRAL TERM (I) + ANTI-WINDUP ---
        if (is_heating_active || is_cooling_active) {
            float integration_step = raw_error * INTEGRAL_GAIN;
            bool apply_step = true;

            // --- ANTI-WINDUP PROTECTION (SATURATION) ---
            const float DELTA_LIMIT_EPS = 0.01f;

            // 1. Calculate prospective integral and error
            float prospective_integral = this->integral_error_z1_ + integration_step;
            float prospective_error = raw_error + prospective_integral;

            // 2. Manage sign (Crucial for profile shaping)
            float prospective_error_sign = (prospective_error > 0.0f) ? 1.0f : ((prospective_error < 0.0f) ? -1.0f : 0.0f);

            // 3. Calculate prospective profile based on absolute value
            float prospective_error_abs = fabs(prospective_error);
            float prospective_error_normalized = prospective_error_abs / max_error_range;
            float prospective_x = fmin(prospective_error_normalized, 1.0f);

            bool use_linear_error_local = (heating_type_index % 2 != 0);
            float prospective_profile = use_linear_error_local ? prospective_x : prospective_x * prospective_x * (3.0f - 2.0f * prospective_x);

            // 4. Reapply sign
            float prospective_error_factor = prospective_profile * prospective_error_sign;

            // 5. Calculate prospective Delta T
            float prospective_dynamic_min_delta_t = base_min_delta_t + (cold_factor * (min_delta_cold_limit - base_min_delta_t));
            float prospective_target_delta_t = prospective_dynamic_min_delta_t + prospective_error_factor * (max_delta_t - prospective_dynamic_min_delta_t);

            // 6. Anti-Windup Logic Checks
            // Case 1: High Saturation (Exceeding max and still rising)
            if (prospective_target_delta_t >= (max_delta_t - DELTA_LIMIT_EPS) && integration_step > 0) {
                ESP_LOGD(OPTIMIZER_TAG, "Anti-Windup: Prospective Delta-T > Max (%.3f), pausing increase.", max_delta_t);
                apply_step = false;
            }
            // Case 2: Low Saturation (Below min and still falling)
            else if (prospective_target_delta_t <= (base_min_delta_t + DELTA_LIMIT_EPS) && integration_step < 0) {
                ESP_LOGD(OPTIMIZER_TAG, "Anti-Windup: Prospective Delta-T < Min (%.3f), pausing decrease.", base_min_delta_t);
                apply_step = false;
            }

            // Flow Temperature Saturation Checks for Heating
            if (is_heating_mode) {
                if (integration_step > 0 && actual_flow_temp >= (zone_max_flow_temp - 0.5f)) {
                    apply_step = false;
                    ESP_LOGD(OPTIMIZER_TAG, "Anti-Windup: Max flow reached (%.1f >= %.1f), pausing integrator increase.", actual_flow_temp, zone_max_flow_temp);
                } else if (integration_step < 0 && actual_flow_temp <= (zone_min_flow_temp + 0.5f)) {
                    apply_step = false;
                    ESP_LOGD(OPTIMIZER_TAG, "Anti-Windup: Min flow reached (%.1f <= %.1f), pausing integrator decrease.", actual_flow_temp, zone_min_flow_temp);
                }
            }
            // --- END ANTI-WINDUP PROTECTION ---

            if (apply_step) {
                this->integral_error_z1_ += integration_step;
                this->integral_error_z1_ = std::clamp(this->integral_error_z1_, -MAX_INTEGRAL_BIAS, MAX_INTEGRAL_BIAS);

                // Save to flash if changed significantly
                if (std::isnan(this->last_saved_integral_) || std::fabs(this->integral_error_z1_ - this->last_saved_integral_) > 0.01f) {
                    this->integral_pref_.save<float>(&this->integral_error_z1_);
                    this->last_saved_integral_ = this->integral_error_z1_;
                }
            }
        }

        // --- 3. FINAL BIAS CALCULATION ---
        // Formula: Bias = (P + I) - D
        // Note: If temperature rises (positive D), we subtract it to reduce the flow temp.
        total_bias += this->integral_error_z1_ - this->derivative_term_;

        this->total_bias_ = total_bias;
        if (this->total_bias_sensor_ != nullptr) {
            this->total_bias_sensor_->publish_state(total_bias);
        }
        
        ESP_LOGD(OPTIMIZER_TAG, "Z1 PID Stats: Error=%.2f, I=%.3f, D=%.3f, Total Bias=%.3f",
                         raw_error, this->integral_error_z1_, this->derivative_term_, total_bias);
    }

    // Apply total bias to the target setpoint
    room_target_temp += total_bias;

    // ==========================================

    ESP_LOGD(OPTIMIZER_TAG, "Processing Zone %d: Room=%.1f, Target=%.1f (Bias %.2f), Actual Flow: %.1f, Return: %.1f, Outside: %.1f, Heat: %d, Cool: %d",
             (i + 1), room_temp, room_target_temp, total_bias, actual_flow_temp, actual_return_temp, actual_outside_temp, is_heating_active, is_cooling_active);

    float error = is_heating_mode ? (room_target_temp - room_temp) : (room_temp - room_target_temp);

    bool use_linear_error = (heating_type_index % 2 != 0);
    float calculated_flow = NAN;

    // Allow signed error: preserve shaping profile but keep sign so
    // negative errors (ambient > setpoint) reduce the resulting delta-T.
    float error_sign = (error > 0.0f) ? 1.0f : ((error < 0.0f) ? -1.0f : 0.0f);
    float error_abs = fabs(error);
    float error_normalized = error_abs / max_error_range;
    float x = fmin(error_normalized, 1.0f);
    float profile = use_linear_error ? x : x * x * (3.0f - 2.0f * x);
    float error_factor = profile * error_sign;

    // Apply cold factor
    float dynamic_min_delta_t = base_min_delta_t + (cold_factor * (min_delta_cold_limit - base_min_delta_t));
    float target_delta_t = dynamic_min_delta_t + error_factor * (max_delta_t - dynamic_min_delta_t);

    // Ensure delta-T never becomes negative (safety) — negative errors
    // should reduce the delta-T toward 0 but not invert it.
    target_delta_t = fmax(target_delta_t, base_min_delta_t);

    // Only update class variable and publish state for Zone 1
    if (i == 0) {
        this->target_delta_t_ = target_delta_t;
        if (this->target_delta_t_sensor_ != nullptr) {
            this->target_delta_t_sensor_->publish_state(this->target_delta_t_);
        }
    }

    ESP_LOGD(OPTIMIZER_TAG, "Effective delta T: %.2f, cold factor: %.2f, dynamic min delta T: %.2f, error factor: %.2f, linear profile: %d",
             target_delta_t, cold_factor, dynamic_min_delta_t, error_factor, use_linear_error);

    if (is_heating_mode && is_heating_active) {
        if (isnan(actual_return_temp)) {
            ESP_LOGE(OPTIMIZER_TAG, "Z%d HEATING (Delta T): FAILED, actual_return_temp is NAN. Reverting to %.2f°C.", (i + 1), zone_min_flow_temp);
            calculated_flow = zone_min_flow_temp;
        } else {
            const float DEFROST_RISK_MIN_TEMP = -2.0f;
            const float DEFROST_RISK_MAX_TEMP = 3.0f;
            const uint32_t DEFROST_MEMORY_MS = 20 * 60 * 1000UL;
            bool defrost_handling_enabled = this->state_.defrost_risk_handling_enabled->state;
            bool is_defrost_weather = false;
            uint32_t current_ms = millis();

            if (actual_outside_temp >= DEFROST_RISK_MIN_TEMP && actual_outside_temp <= DEFROST_RISK_MAX_TEMP) {
                uint32_t last_defrost = this->last_defrost_time_;
                if (last_defrost > 0) {
                    if ((current_ms - last_defrost) < DEFROST_MEMORY_MS) {
                        is_defrost_weather = true;
                    }
                    ESP_LOGD(OPTIMIZER_CYCLE_TAG, "Defrost logic: enabled=%d, active=%d, time_since=%u ms",
                             defrost_handling_enabled, is_defrost_weather, (current_ms - last_defrost));
                }
            }

            if (is_defrost_weather && defrost_handling_enabled) {
                uint32_t elapsed_ms = current_ms - this->last_defrost_time_;
                float recovery_ratio = (float)elapsed_ms / (float)DEFROST_MEMORY_MS;

                recovery_ratio = std::clamp(recovery_ratio, 0.0f, 1.0f);
                float delta_gap = fmax(target_delta_t - base_min_delta_t, 0.0f);
                float ramped_delta_t = base_min_delta_t + (delta_gap * recovery_ratio);

                calculated_flow = actual_return_temp + ramped_delta_t;
                calculated_flow = this->round_nearest(calculated_flow);
                ESP_LOGW(OPTIMIZER_TAG, "Z%d Defrost Recovery: %.0f%% done. Ramp Delta: %.2f (Min: %.2f, Target: %.2f). Flow: %.2f",
                         (i + 1), (recovery_ratio * 100.0f), ramped_delta_t, base_min_delta_t, target_delta_t, calculated_flow);
            } else {
                calculated_flow = actual_return_temp + target_delta_t;
                calculated_flow = this->round_nearest(calculated_flow);

                // If there was a boost adjustment, check if it's still needed and clear if necessary
                float short_cycle_prevention_adjustment = this->predictive_short_cycle_total_adjusted_;
                if (short_cycle_prevention_adjustment > 0.0f) {
                    ESP_LOGD(OPTIMIZER_TAG, "Z%d HEATING (boost adjustment): boost: %.1f°C, calculated_flow: %.2f°C, actual_flow: %.2f°C",
                             (i + 1), short_cycle_prevention_adjustment, calculated_flow, actual_flow_temp);

                    if ((actual_flow_temp - calculated_flow) >= 1.0f) {
                        calculated_flow += short_cycle_prevention_adjustment;
                    } else {
                        this->predictive_short_cycle_total_adjusted_ = 0.0f;
                        short_cycle_prevention_adjustment = 0;
                    }
                }
                ESP_LOGD(OPTIMIZER_TAG, "Z%d HEATING (Delta T): calculated_flow: %.2f°C (boost: %.1f)",
                         (i + 1), calculated_flow, short_cycle_prevention_adjustment);
            }
        }

        calculated_flow = this->clamp_flow_temp(calculated_flow, zone_min_flow_temp, zone_max_flow_temp);
        // Step down limit to avoid compressor halt
        calculated_flow = enforce_step_down(actual_flow_temp, calculated_flow);
        out_flow_heat = calculated_flow;
    } else if (is_cooling_mode && is_cooling_active) {
        ESP_LOGD(OPTIMIZER_TAG, "Using 'Delta-T Control' strategy for cooling.");

        if (isnan(actual_return_temp)) {
            ESP_LOGE(OPTIMIZER_TAG, "Z%d COOLING (Delta T): FAILED, actual_return_temp is NAN. Reverting to smart start temp.");
            calculated_flow = this->state_.cooling_smart_start_temp->state;
        } else {
            calculated_flow = actual_return_temp - target_delta_t;
            ESP_LOGD(OPTIMIZER_TAG, "Z%d COOLING (Delta T): calc: %.1f°C (Return %.1f - Scaled delta T %.1f)",
                     (i + 1), calculated_flow, actual_return_temp, target_delta_t);
        }
        calculated_flow = this->clamp_flow_temp(calculated_flow,
                                                this->state_.minimum_cooling_flow_temp->state,
                                                this->state_.cooling_smart_start_temp->state);
        out_flow_cool = this->round_nearest_half(calculated_flow);
    }
}

void Optimizer::run_auto_adaptive_loop() {
    // Publish total bias at the start of the loop
    if (this->total_bias_sensor_ != nullptr) {
        this->total_bias_sensor_->publish_state(this->total_bias_);
    }

    if (!this->state_.auto_adaptive_control_enabled->state)
        return;

    auto &status = this->state_.ecodan_instance->get_status();

    if (this->is_system_hands_off(status)) {
        ESP_LOGD(OPTIMIZER_TAG, "System is busy (DHW, Defrost, or Lockout). Exiting.");
        return;
    }

    if (status.HeatingCoolingMode != esphome::ecodan::Status::HpMode::HEAT_FLOW_TEMP &&
        status.HeatingCoolingMode != esphome::ecodan::Status::HpMode::COOL_FLOW_TEMP) {
        ESP_LOGD(OPTIMIZER_TAG, "Zone 1 is not in a fixed flow mode. Exiting.");
        return;
    }

    if (status.has_2zones() &&
        (status.HeatingCoolingModeZone2 != esphome::ecodan::Status::HpMode::HEAT_FLOW_TEMP &&
         status.HeatingCoolingModeZone2 != esphome::ecodan::Status::HpMode::COOL_FLOW_TEMP)) {
        ESP_LOGD(OPTIMIZER_TAG, "Zone 2 is not in a fixed flow mode. Exiting.");
        return;
    }

    if (isnan(this->state_.hp_feed_temp->state) || isnan(status.OutsideTemperature)) {
        ESP_LOGW(OPTIMIZER_TAG, "Sensor data unavailable. Exiting.");
        return;
    }

    float actual_outside_temp = status.OutsideTemperature;
    if (isnan(actual_outside_temp))
        return;

    auto heating_type_index = this->state_.heating_system_type->active_index().value_or(0);
    float base_min_delta_t, max_delta_t, max_error_range, min_delta_cold_limit;

    if (heating_type_index <= 1) {
        // UFH (Underfloor Heating)
        base_min_delta_t = 2.0f;
        min_delta_cold_limit = 4.0f;
        max_delta_t = 8.0f;
        max_error_range = 2.0f;
    } else if (heating_type_index <= 3) {
        // Hybrid
        base_min_delta_t = 3.0f;
        min_delta_cold_limit = 5.0f;
        max_delta_t = 8.0f;
        max_error_range = 2.0f;
    } else {
        // Radiator
        base_min_delta_t = 4.0f;
        min_delta_cold_limit = 6.0f;
        max_delta_t = 10.0f;
        max_error_range = 1.5f;
    }

    const float MILD_WEATHER_TEMP = 15.0f;
    const float COLD_WEATHER_TEMP = -5.0f;
    float clamped_outside_temp = std::clamp(actual_outside_temp, COLD_WEATHER_TEMP, MILD_WEATHER_TEMP);
    float cold_factor = (MILD_WEATHER_TEMP - clamped_outside_temp) / (MILD_WEATHER_TEMP - COLD_WEATHER_TEMP);

    this->cold_factor_ = cold_factor;
    if (this->cold_factor_sensor_ != nullptr) {
        this->cold_factor_sensor_->publish_state(this->cold_factor_);
    }

    ESP_LOGD(OPTIMIZER_TAG, "[*] Starting auto-adaptive cycle, z2 independent: %d, has_cooling: %d, cold factor: %.2f, min delta T: %.2f, max delta T: %.2f",
             status.has_independent_z2(), status.has_cooling(), cold_factor, base_min_delta_t, max_delta_t);

    float calculated_flows_heat[2] = {0.0f, 0.0f};
    float calculated_flows_cool[2] = {100.0f, 100.0f};

    auto max_zones = status.has_2zones() ? 2 : 1;

    float max_flow_z1 = this->state_.maximum_heating_flow_temp->state;
    float min_flow_z1 = this->state_.minimum_heating_flow_temp->state;
    if (min_flow_z1 > max_flow_z1) {
        ESP_LOGW(OPTIMIZER_TAG, "Zone 1 Min/Max conflict. Forcing Min (%.1f) to Max (%.1f)", min_flow_z1, max_flow_z1);
        min_flow_z1 = max_flow_z1;
    }

    float max_flow_z2 = max_flow_z1;
    float min_flow_z2 = min_flow_z1;

    if (status.has_2zones()) {
        max_flow_z2 = this->state_.maximum_heating_flow_temp_z2->state;
        min_flow_z2 = this->state_.minimum_heating_flow_temp_z2->state;
        if (min_flow_z2 > max_flow_z2) {
            ESP_LOGW(OPTIMIZER_TAG, "Zone 2 Min/Max conflict. Forcing Min (%.1f) to Max (%.1f)", min_flow_z2, max_flow_z2);
            min_flow_z2 = max_flow_z2;
        }
    }

    for (std::size_t i = 0; i < max_zones; i++) {
        this->process_adaptive_zone_(
            i,
            status,
            cold_factor,
            min_delta_cold_limit,
            base_min_delta_t,
            max_delta_t,
            max_error_range,
            actual_outside_temp,
            (i == 0) ? max_flow_z1 : max_flow_z2,
            (i == 0) ? min_flow_z1 : min_flow_z2,
            calculated_flows_heat[i],
            calculated_flows_cool[i]);
    }

    bool is_heating_demand = calculated_flows_heat[0] > 0.0f || calculated_flows_heat[1] > 0.0f;
    bool is_cooling_demand = calculated_flows_cool[0] < 100.0f || calculated_flows_cool[1] < 100.0f;

    if (status.has_independent_z2()) {
        if (is_heating_demand) {
            if (status.is_auto_adaptive_heating(esphome::ecodan::Zone::ZONE_1) &&
                std::abs(status.Zone1FlowTemperatureSetPoint - calculated_flows_heat[0]) > FLOW_TEMP_TOLERANCE) {
                ESP_LOGD(OPTIMIZER_TAG, "Update Z1 Heat: Old=%.2f, New=%.2f", status.Zone1FlowTemperatureSetPoint, calculated_flows_heat[0]);
                set_flow_temp(calculated_flows_heat[0], OptimizerZone::ZONE_1);
            }
            if (status.is_auto_adaptive_heating(esphome::ecodan::Zone::ZONE_2) &&
                std::abs(status.Zone2FlowTemperatureSetPoint - calculated_flows_heat[1]) > FLOW_TEMP_TOLERANCE) {
                set_flow_temp(calculated_flows_heat[1], OptimizerZone::ZONE_2);
            }
        } else if (is_cooling_demand) {
            if (status.is_auto_adaptive_cooling(esphome::ecodan::Zone::ZONE_1) &&
                std::abs(status.Zone1FlowTemperatureSetPoint - calculated_flows_cool[0]) > FLOW_TEMP_TOLERANCE) {
                set_flow_temp(calculated_flows_cool[0], OptimizerZone::ZONE_1);
            }
            if (status.is_auto_adaptive_cooling(esphome::ecodan::Zone::ZONE_2) &&
                std::abs(status.Zone2FlowTemperatureSetPoint - calculated_flows_cool[1]) > FLOW_TEMP_TOLERANCE) {
                set_flow_temp(calculated_flows_cool[1], OptimizerZone::ZONE_2);
            }
        }
    } else {
        if (is_heating_demand) {
            float final_flow = fmax(calculated_flows_heat[0], calculated_flows_heat[1]);
            if (std::abs(status.Zone1FlowTemperatureSetPoint - final_flow) > FLOW_TEMP_TOLERANCE) {
                ESP_LOGD(OPTIMIZER_TAG, "Update Single/Shared Heat: Old=%.2f, New=%.2f", status.Zone1FlowTemperatureSetPoint, final_flow);
                set_flow_temp(final_flow, OptimizerZone::SINGLE);
            }
        } else if (is_cooling_demand) {
            float final_flow = fmin(calculated_flows_cool[0], calculated_flows_cool[1]);
            if (std::abs(status.Zone1FlowTemperatureSetPoint - final_flow) > FLOW_TEMP_TOLERANCE) {
                set_flow_temp(final_flow, OptimizerZone::SINGLE);
            }
        }
 
    }
  
    // INSERER LE CODE CI-DESSOUS JUSTE AVANT LA DERNIERE ACCOLADE DE LA FONCTION

    // ========================================================================
    // STRATEGIE SMART GRID : PROTECTION SURCHAUFFE ET INERTIE
    // Source: 
    // 1. Activation Mode OFF (Coupure Compresseur) si T_Amb > T_Cible + Offset (ex: 0.7°C)
    // 2. Désactivation Mode OFF si T_Amb <= T_Cible OU T_Retour < 26°C (Calories épuisées)
    // ========================================================================

    if (this->state_.sg_overheat_offset != nullptr && this->state_.sg_mode_off != nullptr) {
        
        // On ne gère cette sécurité que si la PAC est sensée chauffer (ou en pause chauffage)
        // On évite d'interférer en mode été/clim si non désiré.
        bool monitoring_active = true; 

        if (monitoring_active) {
            float offset_val = this->state_.sg_overheat_offset->state; 
            float current_room = status.Zone1RoomTemperature;
            float current_target = status.Zone1SetTemperature;
            float return_temp = status.HpReturnTemperature;

            // Utilisation de la sonde externe si configurée
            if (this->state_.temperature_feedback_source->active_index().value_or(0) == 1) {
                 current_room = this->state_.temperature_feedback_z1->state;
            }

            // Vérification de la validité des données pour éviter des actions sur des valeurs NAN
            if (!std::isnan(current_room) && !std::isnan(current_target) && !std::isnan(offset_val) && !std::isnan(return_temp)) {
                
                // Condition de DÉCLENCHEMENT (Trigger)
                // Si la température dépasse la cible + 0.7°C 
                bool condition_trigger_off = current_room > (current_target + offset_val);
                
                // Condition de RELÂCHEMENT (Release)
                // Si la température redescend OU que l'eau du circuit est devenue froide (< 26°C)
                // < 26°C indique que l'inertie a été transférée aux murs, inutile de circuler à froid.
                bool condition_release_off = (current_room <= current_target) || (return_temp < 26.0f);

                bool is_off_active = this->state_.sg_mode_off->state;

                if (condition_trigger_off && !is_off_active) {
                    ESP_LOGW(OPTIMIZER_TAG, "SmartGrid: Surchauffe détectée (Room %.2f > Target %.2f + Offset %.1f). Activation MODE OFF.", 
                             current_room, current_target, offset_val);
                    this->state_.sg_mode_off->turn_on();
                } 
                else if (condition_release_off && is_off_active) {
                    ESP_LOGI(OPTIMIZER_TAG, "SmartGrid: Fin de surchauffe ou eau froide (Retour %.1f < 26.0). Désactivation MODE OFF.", return_temp);
                    this->state_.sg_mode_off->turn_off();
                }
            }
        }
    }
} // Fin de run_auto_adaptive_loop
} // namespace optimizer
} // namespace esphome
