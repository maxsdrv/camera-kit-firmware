/**
 * @file telemetry_system.cpp
 * @brief Telemetry system implementation
 * @version 1.0.0
 * @date 2025-01-04
 */

#include "telemetry_system.h"
#include "../board/Inc/stm32_hal_config.h"
#include <algorithm>
#include <cstring>

namespace DOF {

TelemetrySystem::TelemetrySystem()
    : max_history_size_(100)
    , update_rate_hz_(100)
    , last_update_time_(0)
    , update_count_(0)
    , error_count_(0)
    , start_time_(0)
    , next_callback_id_(1)
    , logging_enabled_(false)
    , statistics_enabled_(true)
    , initialized_(false)
    , average_latency_(0.0f)
    , latency_samples_(0)
    , total_latency_(0)
{
    memset(&current_telemetry_, 0, sizeof(current_telemetry_));

    // Initialize default values
    current_telemetry_.system_status = 0;
    current_telemetry_.cpu_usage = 0.0f;
    current_telemetry_.free_memory = 0;
    current_telemetry_.error_flags = 0;

    // Initialize camera defaults
    current_telemetry_.camera.pitch_angle = 90.0f;
    current_telemetry_.camera.roll_angle = 90.0f;
    current_telemetry_.camera.zoom_level = 50;
    current_telemetry_.camera.is_recording = false;
    current_telemetry_.camera.frame_count = 0;

    // Initialize network defaults
    current_telemetry_.network.ethernet_connected = false;
    current_telemetry_.network.wifi_connected = false;
    current_telemetry_.network.packets_sent = 0;
    current_telemetry_.network.packets_received = 0;
    current_telemetry_.network.signal_strength = 0;

    // Initialize statistics
    current_telemetry_.statistics.uptime_seconds = 0;
    current_telemetry_.statistics.total_movements = 0;
    current_telemetry_.statistics.emergency_stops = 0;
    current_telemetry_.statistics.average_response_time = 0.0f;
}

void TelemetrySystem::initialize()
{
    if (initialized_) {
        return;
    }

    start_time_ = HAL_GetTick();
    last_update_time_ = start_time_;

    // Reserve space for history to prevent reallocations
    history_.reserve(max_history_size_);

    // Initialize telemetry data
    current_telemetry_.timestamp = start_time_;
    current_telemetry_.system_status = 1; // System initialized bit
    current_telemetry_.error_flags = 0;

    initialized_ = true;
}

void TelemetrySystem::update(const TelemetryArray& servo_telemetry)
{
    if (!initialized_) {
        return;
    }

    uint32_t update_start_time = HAL_GetTick();
    uint32_t update_interval = 1000 / update_rate_hz_;

    // Check if it's time for an update
    if (update_start_time - last_update_time_ < update_interval) {
        return;
    }

    // Update servo data
    current_telemetry_.servo_data = servo_telemetry;
    current_telemetry_.timestamp = update_start_time;

    // Update system metrics
    updateSystemStatus();
    updateCpuUsage();
    updateMemoryUsage();

    if (statistics_enabled_) {
        updateStatistics();
    }

    // Calculate update latency
    calculateLatency(update_start_time);

    // Add to history if logging enabled
    if (logging_enabled_) {
        addToHistory();
    }

    // Notify callbacks
    notifyCallbacks();

    last_update_time_ = update_start_time;
    update_count_++;
}

void TelemetrySystem::shutdown()
{
    if (!initialized_) {
        return;
    }

    callbacks_.clear();
    history_.clear();
    initialized_ = false;

    current_telemetry_.system_status &= ~0x01; // Clear initialized bit
}

std::vector<SystemTelemetry> TelemetrySystem::getHistory(size_t count) const
{
    if (history_.empty()) {
        return {};
    }

    size_t start_index = 0;
    if (count < history_.size()) {
        start_index = history_.size() - count;
    }

    return std::vector<SystemTelemetry>(
        history_.begin() + start_index,
        history_.end()
    );
}

void TelemetrySystem::addCallback(TelemetryCallback callback)
{
    if (callback) {
        callbacks_.emplace_back(next_callback_id_++, callback);
    }
}

void TelemetrySystem::removeCallback(size_t callback_id)
{
    callbacks_.erase(
        std::remove_if(callbacks_.begin(), callbacks_.end(),
            [callback_id](const auto& pair) {
                return pair.first == callback_id;
            }),
        callbacks_.end()
    );
}

void TelemetrySystem::setUpdateRate(uint32_t rate_hz)
{
    if (rate_hz > 0 && rate_hz <= 1000) {
        update_rate_hz_ = rate_hz;
    }
}

void TelemetrySystem::setHistorySize(size_t size)
{
    max_history_size_ = size;
    if (history_.size() > max_history_size_) {
        trimHistory();
    }
}

void TelemetrySystem::enableLogging(bool enable)
{
    logging_enabled_ = enable;
    if (!enable) {
        history_.clear();
    }
}

void TelemetrySystem::enableStatistics(bool enable)
{
    statistics_enabled_ = enable;
    if (!enable) {
        resetStatistics();
    }
}

uint32_t TelemetrySystem::getUptimeSeconds() const
{
    if (!initialized_) {
        return 0;
    }
    return (HAL_GetTick() - start_time_) / 1000;
}

void TelemetrySystem::updateCameraData(float pitch, float roll, uint8_t zoom, bool recording)
{
    current_telemetry_.camera.pitch_angle = pitch;
    current_telemetry_.camera.roll_angle = roll;
    current_telemetry_.camera.zoom_level = zoom;
    current_telemetry_.camera.is_recording = recording;

    if (recording) {
        current_telemetry_.camera.frame_count++;
    }
}

void TelemetrySystem::updateNetworkStatus(bool eth_conn, bool wifi_conn, uint16_t signal)
{
    current_telemetry_.network.ethernet_connected = eth_conn;
    current_telemetry_.network.wifi_connected = wifi_conn;
    current_telemetry_.network.signal_strength = signal;

    if (eth_conn || wifi_conn) {
        current_telemetry_.network.packets_sent++;
    }
}

void TelemetrySystem::incrementMovementCount()
{
    current_telemetry_.statistics.total_movements++;
}

void TelemetrySystem::incrementEmergencyStopCount()
{
    current_telemetry_.statistics.emergency_stops++;
}

void TelemetrySystem::updateSystemStatus()
{
    // System status bits:
    // Bit 0: Initialized
    // Bit 1: Healthy (no critical errors)
    // Bit 2: Emergency stop active
    // Bit 3: Network connected
    // Bit 4: Camera active
    // Bit 5-7: Reserved

    uint8_t status = 0;

    if (initialized_) {
        status |= 0x01;
    }

    // Check if system is healthy (no critical errors in servo data)
    bool system_healthy = true;
    for (const auto& servo : current_telemetry_.servo_data) {
        if (servo.error != ServoError::NONE) {
            system_healthy = false;
            break;
        }
    }
    if (system_healthy && error_count_ == 0) {
        status |= 0x02;
    }

    // Check network connection
    if (current_telemetry_.network.ethernet_connected || 
        current_telemetry_.network.wifi_connected) {
        status |= 0x08;
    }

    // Check camera status
    if (current_telemetry_.camera.is_recording) {
        status |= 0x10;
    }

    current_telemetry_.system_status = status;
}

void TelemetrySystem::updateCpuUsage()
{
    current_telemetry_.cpu_usage = calculateCpuUsage();
}

void TelemetrySystem::updateMemoryUsage()
{
    current_telemetry_.free_memory = getFreeMemoryBytes();
}

void TelemetrySystem::updateStatistics()
{
    current_telemetry_.statistics.uptime_seconds = getUptimeSeconds();
    current_telemetry_.statistics.average_response_time = average_latency_;
}

void TelemetrySystem::calculateLatency(uint32_t start_time)
{
    uint32_t end_time = HAL_GetTick();
    uint32_t latency = end_time - start_time;

    total_latency_ += latency;
    latency_samples_++;

    if (latency_samples_ > 0) {
        average_latency_ = static_cast<float>(total_latency_) / latency_samples_;
    }

    // Reset every 1000 samples to prevent overflow
    if (latency_samples_ >= 1000) {
        total_latency_ = static_cast<uint32_t>(average_latency_);
        latency_samples_ = 1;
    }
}

void TelemetrySystem::notifyCallbacks()
{
    for (auto it = callbacks_.begin(); it != callbacks_.end(); ++it) {
        // Call callback function
        it->second(current_telemetry_);
    }
}

void TelemetrySystem::addToHistory()
{
    history_.push_back(current_telemetry_);

    if (history_.size() > max_history_size_) {
        trimHistory();
    }
}

void TelemetrySystem::trimHistory()
{
    if (history_.size() > max_history_size_) {
        size_t elements_to_remove = history_.size() - max_history_size_;
        history_.erase(history_.begin(), history_.begin() + elements_to_remove);
    }
}

uint16_t TelemetrySystem::getFreeMemoryBytes() const
{
    // Simple stack pointer check for available memory
    // This is a basic implementation - in production you might want more sophisticated memory tracking
    extern char _end;
    char *heapend = &_end;
    char *stack_ptr = (char*)__get_MSP();

    return static_cast<uint16_t>(stack_ptr - heapend);
}

float TelemetrySystem::calculateCpuUsage() const
{
    // Simple CPU usage estimation based on update timing
    // This is a basic implementation - you could implement more sophisticated CPU monitoring
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t delta_time = current_time - last_time;
    last_time = current_time;

    if (delta_time == 0) {
        return current_telemetry_.cpu_usage; // Return previous value
    }

    // Estimate CPU usage based on how close we are to our target update rate
    uint32_t expected_interval = 1000 / update_rate_hz_;
    float cpu_usage = std::min(100.0f, (static_cast<float>(expected_interval) / delta_time) * 100.0f);

    return cpu_usage;
}

void TelemetrySystem::resetStatistics()
{
    current_telemetry_.statistics.total_movements = 0;
    current_telemetry_.statistics.emergency_stops = 0;
    current_telemetry_.statistics.average_response_time = 0.0f;

    average_latency_ = 0.0f;
    latency_samples_ = 0;
    total_latency_ = 0;
}

} // namespace DOF
