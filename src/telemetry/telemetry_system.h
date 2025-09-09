/**
 * @file telemetry_system.h
 * @brief Real-time telemetry system for DOF robotic arm
 * @version 1.0.0
 * @date 2025-01-04
 */

#pragma once

#include "../gimbal/types/servo_types.h"
#include <array>
#include <functional>
#include <memory>
#include <vector>

namespace DOF {

/**
 * @brief Comprehensive system telemetry data structure
 * 
 * Contains all telemetry data for the DOF robotic arm system including:
 * - Servo positions and status
 * - System health metrics
 * - Camera data
 * - Network status
 * - Performance statistics
 */
struct SystemTelemetry {
    TelemetryArray servo_data;
    uint32_t timestamp;
    uint8_t system_status;
    float cpu_usage;
    uint16_t free_memory;
    uint8_t error_flags;

    // Camera-specific data
    struct Camera {
        float pitch_angle;
        float roll_angle;
        uint8_t zoom_level;
        bool is_recording;
        uint32_t frame_count;
    } camera;

    // Network status (for future expansion)
    struct Network {
        bool ethernet_connected;
        bool wifi_connected;
        uint32_t packets_sent;
        uint32_t packets_received;
        uint16_t signal_strength;
    } network;

    // System statistics
    struct Statistics {
        uint32_t uptime_seconds;
        uint32_t total_movements;
        uint32_t emergency_stops;
        float average_response_time;
    } statistics;
};

/**
 * @brief Real-time telemetry system for comprehensive monitoring
 * 
 * Features:
 * - 100Hz telemetry updates
 * - Historical data storage
 * - Callback system for real-time notifications
 * - Performance metrics
 * - System health monitoring
 */
class TelemetrySystem {
public:
    using TelemetryCallback = std::function<void(const SystemTelemetry&)>;

    TelemetrySystem();
    ~TelemetrySystem() = default;

    // Core functionality
    void initialize();
    void update(const TelemetryArray& servo_telemetry);
    void shutdown();

    // Data access
    const SystemTelemetry& getCurrentTelemetry() const { return current_telemetry_; }
    std::vector<SystemTelemetry> getHistory(size_t count) const;

    // Callback management
    void addCallback(TelemetryCallback callback);
    void removeCallback(size_t callback_id);

    // Configuration
    void setUpdateRate(uint32_t rate_hz);
    void setHistorySize(size_t size);
    void enableLogging(bool enable);
    void enableStatistics(bool enable);

    // Statistics and metrics
    float getAverageLatency() const { return average_latency_; }
    uint32_t getUpdateCount() const { return update_count_; }
    uint32_t getErrorCount() const { return error_count_; }
    uint32_t getUptimeSeconds() const;

    // Manual data updates for camera and network
    void updateCameraData(float pitch, float roll, uint8_t zoom, bool recording);
    void updateNetworkStatus(bool eth_conn, bool wifi_conn, uint16_t signal);
    void incrementMovementCount();
    void incrementEmergencyStopCount();

private:
    // Core data
    SystemTelemetry current_telemetry_;
    std::vector<SystemTelemetry> history_;
    std::vector<std::pair<size_t, TelemetryCallback>> callbacks_;

    // Configuration
    size_t max_history_size_;
    uint32_t update_rate_hz_;
    uint32_t last_update_time_;
    uint32_t update_count_;
    uint32_t error_count_;
    uint32_t start_time_;
    size_t next_callback_id_;

    // Status flags
    bool logging_enabled_;
    bool statistics_enabled_;
    bool initialized_;

    // Performance metrics
    float average_latency_;
    uint32_t latency_samples_;
    uint32_t total_latency_;

    // Internal methods
    void updateSystemStatus();
    void updateCpuUsage();
    void updateMemoryUsage();
    void updateStatistics();
    void calculateLatency(uint32_t start_time);
    void notifyCallbacks();
    void addToHistory();
    void trimHistory();

    // Helper methods
    uint16_t getFreeMemoryBytes() const;
    float calculateCpuUsage() const;
    void resetStatistics();
};

} // namespace DOF
