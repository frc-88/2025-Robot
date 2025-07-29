package frc.robot.health;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionConstants;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Central CAN health monitor for collecting device statuses and analyzing CAN bus health.
 * 
 * This singleton class provides real-time monitoring of CAN device connectivity,
 * generates alerts for various failure patterns, and integrates with logging systems.
 */
public class CANHealthMonitor {
    // Singleton instance
    private static CANHealthMonitor instance;

    // Alert timing constants (in seconds)
    private static final double IMMEDIATE_ALERT_THRESHOLD = 1.0;
    private static final double SUSTAINED_ALERT_THRESHOLD = 3.0;
    private static final double INTERMITTENT_WINDOW = 10.0;
    private static final int INTERMITTENT_THRESHOLD = 3; // 3+ disconnects in 10 seconds

    // Thread-safe storage for device statuses and events
    private final Map<String, Boolean> currentDeviceStatus = new ConcurrentHashMap<>();
    private final Map<String, Queue<StatusEvent>> deviceEvents = new ConcurrentHashMap<>();
    private final Map<String, ActiveAlert> activeAlerts = new ConcurrentHashMap<>();

    /**
     * Defines the physical wiring order for each CAN bus.
     * Device keys must match the strings passed to updateStatus().
     */
    private final Map<String, List<String>> wiringOrder = Map.of(
        "rio", List.of(
            "Climber/Gripper",
            "Climber/GasMotor", 
            "Climber/Encoder",
            "Climber/CANRange",
            "Doghouse/FunnelMotor",
            "Doghouse/ManipulatorMotor",
            "Doghouse/DoghouseCANRange",
            "Doghouse/CoralCANRange",
            "Doghouse/ReefCANRange",
            "Armevator/ElevatorMain",
            "Armevator/ElevatorFollower",
            "Armevator/Arm",
            "Armevator/Encoder",
            "Lights/CANdle"
        ),
        "Canivore", List.of(
            "Drive/FrontRight/DriveMotor",
            "Drive/FrontRight/TurnEncoder", 
            "Drive/FrontRight/TurnMotor",
            "Drive/FrontLeft/DriveMotor",
            "Drive/FrontLeft/TurnEncoder",
            "Drive/FrontLeft/TurnMotor",
            "Drive/BackLeft/DriveMotor",
            "Drive/BackLeft/TurnEncoder",
            "Drive/BackLeft/TurnMotor",
            "Drive/BackRight/DriveMotor",
            "Drive/BackRight/TurnEncoder",
            "Drive/BackRight/TurnMotor",
            "Drive/Gyro"
        )
    );

    /**
     * Maps CAN IDs to human-readable device names.
     * Auto-generated from TunerConstants to stay in sync with CAN ID changes.
     */
    private static final Map<Integer, String> CAN_ID_TO_DEVICE_NAME = buildCanIdMap();

    private static Map<Integer, String> buildCanIdMap() {
        Map<Integer, String> map = new HashMap<>();
        
        // Auto-generate from TunerConstants
        map.put(TunerConstants.FrontLeft.DriveMotorId, "Drive/FrontLeft/DriveMotor");
        map.put(TunerConstants.FrontLeft.SteerMotorId, "Drive/FrontLeft/TurnMotor");
        map.put(TunerConstants.FrontLeft.EncoderId, "Drive/FrontLeft/TurnEncoder");
        
        map.put(TunerConstants.FrontRight.DriveMotorId, "Drive/FrontRight/DriveMotor");
        map.put(TunerConstants.FrontRight.SteerMotorId, "Drive/FrontRight/TurnMotor");
        map.put(TunerConstants.FrontRight.EncoderId, "Drive/FrontRight/TurnEncoder");
        
        map.put(TunerConstants.BackLeft.DriveMotorId, "Drive/BackLeft/DriveMotor");
        map.put(TunerConstants.BackLeft.SteerMotorId, "Drive/BackLeft/TurnMotor");
        map.put(TunerConstants.BackLeft.EncoderId, "Drive/BackLeft/TurnEncoder");
        
        map.put(TunerConstants.BackRight.DriveMotorId, "Drive/BackRight/DriveMotor");
        map.put(TunerConstants.BackRight.SteerMotorId, "Drive/BackRight/TurnMotor");
        map.put(TunerConstants.BackRight.EncoderId, "Drive/BackRight/TurnEncoder");
        
        // Add gyro CAN ID if available in TunerConstants
        map.put(TunerConstants.DrivetrainConstants.Pigeon2Id, "Drive/Gyro");
        
        return map;
    }

    /**
     * Maps generic device keys to human-readable names from constants.
     * Used for better device identification in alerts and diagnostics.
     */
    private static final Map<String, String> DEVICE_NAME_MAPPINGS = Map.of(
        // Vision cameras - map to VisionConstants names
        "Vision/Camera0", VisionConstants.camera0Name,  // "limelight-back"
        "Vision/Camera1", VisionConstants.camera1Name   // "limelight-front"
        // Could add more mappings here for other subsystems if needed
    );

    /**
     * Represents a status change event for a CAN device.
     */
    private static class StatusEvent {
        public final double timestamp;
        public final boolean connected;

        public StatusEvent(boolean connected) {
            this.timestamp = RobotController.getFPGATime() / 1e6; // Convert microseconds to seconds
            this.connected = connected;
        }
    }

    /**
     * Represents an active alert for a CAN device.
     */
    public static class ActiveAlert {
        public final String deviceKey;
        public final AlertType alertType;
        public final double startTime;
        public double lastSeen;
        public boolean cleared;

        public ActiveAlert(String deviceKey, AlertType alertType) {
            this.deviceKey = deviceKey;
            this.alertType = alertType;
            this.startTime = RobotController.getFPGATime() / 1e6; // Convert microseconds to seconds
            this.lastSeen = this.startTime;
            this.cleared = false;
        }

        public void updateLastSeen() {
            this.lastSeen = RobotController.getFPGATime() / 1e6; // Convert microseconds to seconds
        }

        public double getDuration() {
            return (RobotController.getFPGATime() / 1e6) - startTime; // Convert microseconds to seconds
        }

        @Override
        public String toString() {
            return String.format("%s: %s (%.1fs)", deviceKey, alertType, getDuration());
        }
    }

    /**
     * Types of alerts that can be generated.
     */
    public enum AlertType {
        IMMEDIATE("Device disconnected"),
        SUSTAINED("Device offline for extended period"), 
        INTERMITTENT("Device connection unstable");

        public final String description;

        AlertType(String description) {
            this.description = description;
        }
    }

    /**
     * Alert severity levels for dashboard display.
     */
    public enum AlertLevel {
        NONE("All CAN devices OK"),
        LOW("Minor CAN issues detected"),
        MEDIUM("CAN device problems detected"),
        HIGH("Critical CAN failures detected");

        public final String description;

        AlertLevel(String description) {
            this.description = description;
        }
    }

    // Private constructor to enforce singleton pattern
    private CANHealthMonitor() {
        // Initialize empty queues for all known devices
        for (List<String> devices : wiringOrder.values()) {
            for (String device : devices) {
                deviceEvents.put(device, new LinkedList<>());
                currentDeviceStatus.put(device, false); // Start assuming disconnected
            }
        }
    }

    /**
     * Returns the single instance of the health monitor.
     */
    public static CANHealthMonitor getInstance() {
        if (instance == null) {
            instance = new CANHealthMonitor();
        }
        return instance;
    }

    /**
     * Returns a human-readable device name, using constants-based mapping if available.
     */
    private String getDisplayName(String deviceKey) {
        return DEVICE_NAME_MAPPINGS.getOrDefault(deviceKey, deviceKey);
    }

    /**
     * Updates the connection status for a CAN device.
     * This should be called from each subsystem's periodic() method.
     *
     * @param deviceKey Unique identifier for the device (e.g., "Drive/FrontLeft/DriveMotor" or "CANID_1")
     * @param isConnected Current connection status of the device
     */
    public void updateStatus(String deviceKey, boolean isConnected) {
        // Translate CAN ID keys to human names
        if (deviceKey.startsWith("CANID_")) {
            int canId = Integer.parseInt(deviceKey.substring(6));
            String humanName = CAN_ID_TO_DEVICE_NAME.get(canId);
            if (humanName != null) {
                deviceKey = humanName;
            }
        }
        Boolean previousStatus = currentDeviceStatus.get(deviceKey);
        
        // Initialize device if not seen before
        if (previousStatus == null) {
            deviceEvents.put(deviceKey, new LinkedList<>());
            previousStatus = false;
        }

        // Update current status
        currentDeviceStatus.put(deviceKey, isConnected);

        // Record event if status changed
        if (previousStatus != isConnected) {
            Queue<StatusEvent> events = deviceEvents.get(deviceKey);
            events.offer(new StatusEvent(isConnected));
            
            // Clean up old events (keep only last 10 seconds)
            cleanupOldEvents(events);
            
            // Log status change using display name for better readability
            String displayName = getDisplayName(deviceKey);
            Logger.recordOutput("CANHealth/Events/" + displayName.replace("/", "_").replace("-", "_"), isConnected);
            
            // Check for new alerts
            checkForAlerts(deviceKey, events);
        }

        // Update existing alerts
        updateExistingAlerts(deviceKey, isConnected);
    }

    /**
     * Updates the connection status for a device that uses Phoenix 6 BaseStatusSignal.
     */
    public void updateStatus(String deviceKey, Object device) {
        if (device instanceof com.ctre.phoenix.led.CANdle) {
            // CANdle doesn't have isConnected method, assume connected if object exists
            updateStatus(deviceKey, device != null);
        } else {
            // For other Phoenix devices, try to call isConnected() via reflection
            try {
                java.lang.reflect.Method method = device.getClass().getMethod("isConnected");
                boolean isConnected = (boolean) method.invoke(device);
                updateStatus(deviceKey, isConnected);
            } catch (Exception e) {
                // If reflection fails, assume connected
                updateStatus(deviceKey, true);
            }
        }
    }

    /**
     * Removes old events from the queue to keep memory usage bounded.
     */
    private void cleanupOldEvents(Queue<StatusEvent> events) {
        double currentTime = RobotController.getFPGATime() / 1e6; // Convert microseconds to seconds
        while (!events.isEmpty() && 
               (currentTime - events.peek().timestamp) > INTERMITTENT_WINDOW) {
            events.poll();
        }
    }

    /**
     * Analyzes recent events to detect alert conditions.
     */
    private void checkForAlerts(String deviceKey, Queue<StatusEvent> events) {
        double currentTime = RobotController.getFPGATime() / 1e6; // Convert microseconds to seconds
        
        // Count recent disconnect events
        int disconnectCount = 0;
        for (StatusEvent event : events) {
            if (!event.connected && (currentTime - event.timestamp) <= INTERMITTENT_WINDOW) {
                disconnectCount++;
            }
        }

        // Check for intermittent connection issues
        if (disconnectCount >= INTERMITTENT_THRESHOLD) {
            createAlert(deviceKey, AlertType.INTERMITTENT);
        }
        
        // Check for immediate disconnect
        StatusEvent latestEvent = ((LinkedList<StatusEvent>) events).peekLast();
        if (latestEvent != null && !latestEvent.connected) {
            createAlert(deviceKey, AlertType.IMMEDIATE);
        }
    }

    /**
     * Updates timing on existing alerts and promotes immediate alerts to sustained.
     */
    private void updateExistingAlerts(String deviceKey, boolean isConnected) {
        ActiveAlert alert = activeAlerts.get(deviceKey);
        if (alert != null && !alert.cleared) {
            alert.updateLastSeen();
            
            // Promote immediate alert to sustained if device still disconnected
            if (alert.alertType == AlertType.IMMEDIATE && !isConnected &&
                alert.getDuration() >= SUSTAINED_ALERT_THRESHOLD) {
                
                activeAlerts.remove(deviceKey);
                createAlert(deviceKey, AlertType.SUSTAINED);
            }
        }
    }

    /**
     * Creates a new alert for a device.
     */
    private void createAlert(String deviceKey, AlertType alertType) {
        ActiveAlert existingAlert = activeAlerts.get(deviceKey);
        
        // Don't create duplicate alerts or downgrade alert severity
        if (existingAlert != null && !existingAlert.cleared) {
            if (existingAlert.alertType == AlertType.SUSTAINED || 
                existingAlert.alertType == AlertType.INTERMITTENT) {
                return; // Don't downgrade from sustained/intermittent to immediate
            }
        }

        ActiveAlert newAlert = new ActiveAlert(deviceKey, alertType);
        activeAlerts.put(deviceKey, newAlert);
        
        // Log alert creation using display name
        String displayName = getDisplayName(deviceKey);
        Logger.recordOutput("CANHealth/Alerts/" + displayName.replace("/", "_").replace("-", "_") + "_" + alertType, true);
        
        // Update dashboard
        updateDashboard();
    }

    /**
     * Returns the highest severity level among all active alerts.
     */
    public AlertLevel getCurrentMaxSeverity() {
        AlertLevel maxLevel = AlertLevel.NONE;
        
        for (ActiveAlert alert : activeAlerts.values()) {
            if (alert.cleared) continue;
            
            AlertLevel alertLevel = switch (alert.alertType) {
                case IMMEDIATE -> AlertLevel.MEDIUM;
                case SUSTAINED -> AlertLevel.HIGH;
                case INTERMITTENT -> AlertLevel.LOW;
            };
            
            if (alertLevel.ordinal() > maxLevel.ordinal()) {
                maxLevel = alertLevel;
            }
        }
        
        return maxLevel;
    }

    /**
     * Returns a description of the most severe current alert.
     */
    public String getCurrentMaxAlert() {
        AlertLevel maxLevel = getCurrentMaxSeverity();
        if (maxLevel == AlertLevel.NONE) {
            return "All CAN devices operational";
        }
        
        // Find the alert that corresponds to max severity
        for (ActiveAlert alert : activeAlerts.values()) {
            if (alert.cleared) continue;
            
            AlertLevel alertLevel = switch (alert.alertType) {
                case IMMEDIATE -> AlertLevel.MEDIUM;
                case SUSTAINED -> AlertLevel.HIGH; 
                case INTERMITTENT -> AlertLevel.LOW;
            };
            
            if (alertLevel == maxLevel) {
                String displayName = getDisplayName(alert.deviceKey);
                return String.format("%s: %s (%.1fs)", displayName, alert.alertType, alert.getDuration());
            }
        }
        
        return maxLevel.description;
    }

    /**
     * Returns all currently active (uncleared) alerts.
     */
    public List<ActiveAlert> getAllActiveAlerts() {
        return activeAlerts.values().stream()
            .filter(alert -> !alert.cleared)
            .sorted((a, b) -> Double.compare(b.startTime, a.startTime)) // Most recent first
            .toList();
    }

    /**
     * Returns current connection status for all monitored devices.
     */
    public Map<String, Boolean> getAllDeviceStatus() {
        return new HashMap<>(currentDeviceStatus);
    }

    /**
     * Manually clears an alert. Used by pit crew to acknowledge issues.
     */
    public void clearAlert(String deviceKey) {
        ActiveAlert alert = activeAlerts.get(deviceKey);
        if (alert != null) {
            alert.cleared = true;
            String displayName = getDisplayName(deviceKey);
            Logger.recordOutput("CANHealth/AlertsCleared/" + displayName.replace("/", "_").replace("-", "_"), true);
            updateDashboard();
        }
    }

    /**
     * Clears all active alerts.
     */
    public void clearAllAlerts() {
        for (ActiveAlert alert : activeAlerts.values()) {
            alert.cleared = true;
        }
        Logger.recordOutput("CANHealth/AllAlertsCleared", RobotController.getFPGATime() / 1e6);
        updateDashboard();
    }

    // ====== PHASE 2: ENHANCED DASHBOARD METHODS ======

    /**
     * Gets a simple status message appropriate for driver display during matches.
     * Designed to be minimally distracting while conveying critical information.
     */
    public String getDriverDisplayStatus() {
        AlertLevel severity = getCurrentMaxSeverity();
        return switch(severity) {
            case NONE -> "CAN OK";
            case LOW -> "CAN MINOR";
            case MEDIUM -> "CAN WARN";
            case HIGH -> "CAN CRIT";
        };
    }

    /**
     * Returns whether the driver dashboard should prominently display a CAN alert.
     * Only shows alerts for medium+ severity to avoid distracting drivers with minor issues.
     */
    public boolean shouldShowDriverAlert() {
        return getCurrentMaxSeverity().ordinal() >= AlertLevel.MEDIUM.ordinal();
    }

    /**
     * Gets a brief message describing the most critical current issue for drivers.
     */
    public String getDriverAlertMessage() {
        if (!shouldShowDriverAlert()) {
            return "";
        }
        
        // Find the highest priority active alert
        ActiveAlert criticalAlert = null;
        AlertLevel maxSeverity = AlertLevel.NONE;
        
        for (ActiveAlert alert : activeAlerts.values()) {
            if (alert.cleared) continue;
            
            AlertLevel alertLevel = switch (alert.alertType) {
                case IMMEDIATE -> AlertLevel.MEDIUM;
                case SUSTAINED -> AlertLevel.HIGH;
                case INTERMITTENT -> AlertLevel.LOW;
            };
            
            if (alertLevel.ordinal() > maxSeverity.ordinal()) {
                maxSeverity = alertLevel;
                criticalAlert = alert;
            }
        }
        
        if (criticalAlert == null) {
            return "";
        }
        
        // Return a brief, driver-friendly message
        String subsystem = criticalAlert.deviceKey.split("/")[0];
        return switch (criticalAlert.alertType) {
            case SUSTAINED -> subsystem + " offline";
            case IMMEDIATE -> subsystem + " disconnected";  
            case INTERMITTENT -> subsystem + " unstable";
        };
    }

    /**
     * Returns detailed alert information formatted for pit crew analysis.
     */
    public List<String> getDetailedAlertList() {
        return getAllActiveAlerts().stream()
            .map(alert -> String.format("[%s] %s - %s (%.1fs active)", 
                alert.alertType.name(), 
                alert.deviceKey, 
                alert.alertType.description, 
                alert.getDuration()))
            .collect(java.util.stream.Collectors.toList());
    }

    /**
     * Returns a comprehensive device status summary for pit diagnostics.
     */
    public Map<String, String> getDeviceStatusSummary() {
        Map<String, String> summary = new HashMap<>();
        
        for (Map.Entry<String, Boolean> entry : currentDeviceStatus.entrySet()) {
            String deviceKey = entry.getKey();
            boolean isConnected = entry.getValue();
            ActiveAlert alert = activeAlerts.get(deviceKey);
            
            String status;
            if (isConnected) {
                status = "OK";
            } else {
                status = "DISCONNECTED";
            }
            
            // Add alert information if present
            if (alert != null && !alert.cleared) {
                status += " (" + alert.alertType.name() + ")";
            }
            
            summary.put(deviceKey, status);
        }
        
        return summary;
    }

    /**
     * Groups active alerts by subsystem for organized pit display.
     */
    public Map<String, List<ActiveAlert>> getAlertsBySubsystem() {
        return getAllActiveAlerts().stream()
            .collect(java.util.stream.Collectors.groupingBy(alert -> 
                alert.deviceKey.split("/")[0])); // Group by first part of device key
    }

    /**
     * Returns alerts specific to a subsystem.
     */
    public List<ActiveAlert> getAlertsForSubsystem(String subsystem) {
        return getAllActiveAlerts().stream()
            .filter(alert -> alert.deviceKey.startsWith(subsystem + "/"))
            .collect(java.util.stream.Collectors.toList());
    }

    /**
     * Determines if any critical devices have alerts.
     */
    public boolean hasCriticalAlerts() {
        List<String> criticalDevices = List.of(
            "Drive/FrontLeft/DriveMotor", "Drive/FrontRight/DriveMotor",
            "Drive/BackLeft/DriveMotor", "Drive/BackRight/DriveMotor", 
            "Drive/Gyro",
            "Armevator/ElevatorMain"
        );
        
        return getAllActiveAlerts().stream()
            .anyMatch(alert -> criticalDevices.contains(alert.deviceKey));
    }

    /**
     * Enhanced dashboard update method that publishes to both driver and pit contexts.
     */
    private void updateDashboard() {
        updateDriverDashboard();
        updatePitDashboard();
        handleDashboardCommands();
    }

    /**
     * Updates NetworkTables keys for driver dashboard display.
     */
    private void updateDriverDashboard() {
        AlertLevel severity = getCurrentMaxSeverity();
        
        SmartDashboard.putString("CAN/Driver/Status", getDriverDisplayStatus());
        SmartDashboard.putString("CAN/Driver/Alert", getDriverAlertMessage());
        SmartDashboard.putBoolean("CAN/Driver/ShowAlert", shouldShowDriverAlert());
        SmartDashboard.putBoolean("CAN/Driver/HasCriticalAlerts", hasCriticalAlerts());
        
        // Add color coding hints for dashboard
        SmartDashboard.putString("CAN/Driver/Color", switch(severity) {
            case NONE -> "green";
            case LOW -> "yellow"; 
            case MEDIUM -> "orange";
            case HIGH -> "red";
        });
    }

    /**
     * Updates NetworkTables keys for detailed pit crew dashboard.
     */
    private void updatePitDashboard() {
        // Detailed alert information
        List<String> alertDetails = getDetailedAlertList();
        SmartDashboard.putStringArray("CAN/Pit/AllAlerts", 
            alertDetails.toArray(new String[0]));
        SmartDashboard.putNumber("CAN/Pit/AlertCount", alertDetails.size());
        
        // Individual device statuses for detailed view
        Map<String, String> deviceStatus = getDeviceStatusSummary();
        for (Map.Entry<String, String> entry : deviceStatus.entrySet()) {
            // Replace slashes with underscores for NetworkTables key compatibility
            String ntKey = "CAN/Pit/Devices/" + entry.getKey().replace("/", "_");
            SmartDashboard.putString(ntKey, entry.getValue());
        }
        
        // Subsystem-level summaries
        Map<String, List<ActiveAlert>> alertsBySubsystem = getAlertsBySubsystem();
        for (String subsystem : List.of("Drive", "Armevator", "Climber", "Doghouse", "Lights")) {
            List<ActiveAlert> subsystemAlerts = alertsBySubsystem.getOrDefault(subsystem, List.of());
            SmartDashboard.putNumber("CAN/Pit/Subsystems/" + subsystem + "/AlertCount", 
                subsystemAlerts.size());
            
            // Find max severity for this subsystem
            AlertLevel maxSeverity = subsystemAlerts.stream()
                .map(alert -> switch (alert.alertType) {
                    case IMMEDIATE -> AlertLevel.MEDIUM;
                    case SUSTAINED -> AlertLevel.HIGH;
                    case INTERMITTENT -> AlertLevel.LOW;
                })
                .max(java.util.Comparator.comparing(Enum::ordinal))
                .orElse(AlertLevel.NONE);
            
            SmartDashboard.putString("CAN/Pit/Subsystems/" + subsystem + "/Status", 
                maxSeverity.name());
        }
        
        // Bus-level summaries
        for (Map.Entry<String, List<String>> bus : wiringOrder.entrySet()) {
            String busName = bus.getKey();
            List<String> devices = bus.getValue();
            
            int connected = 0;
            int withAlerts = 0;
            
            for (String device : devices) {
                if (currentDeviceStatus.getOrDefault(device, false)) {
                    connected++;
                }
                ActiveAlert alert = activeAlerts.get(device);
                if (alert != null && !alert.cleared) {
                    withAlerts++;
                }
            }
            
            SmartDashboard.putString("CAN/Pit/Bus/" + busName + "/Summary", 
                connected + "/" + devices.size() + " connected");
            SmartDashboard.putNumber("CAN/Pit/Bus/" + busName + "/AlertCount", withAlerts);
            SmartDashboard.putBoolean("CAN/Pit/Bus/" + busName + "/AllOK", 
                connected == devices.size() && withAlerts == 0);
        }
        
        // Overall statistics
        SmartDashboard.putBoolean("CAN/Pit/HasCriticalAlerts", hasCriticalAlerts());
        SmartDashboard.putString("CAN/Pit/OverallStatus", getCurrentMaxSeverity().name());
        
        // Legacy Phase 1 keys for backwards compatibility
        SmartDashboard.putString("CAN/Status", getCurrentMaxSeverity().name());
        SmartDashboard.putString("CAN/MaxAlert", getCurrentMaxAlert());
        SmartDashboard.putNumber("CAN/ActiveAlertCount", getAllActiveAlerts().size());
        SmartDashboard.putNumber("CAN/DeviceCount", currentDeviceStatus.size());
        SmartDashboard.putNumber("CAN/ConnectedCount", 
            (int) currentDeviceStatus.values().stream().mapToInt(b -> b ? 1 : 0).sum());
    }

    /**
     * Handles commands sent from dashboard (like clear alert buttons).
     */
    private void handleDashboardCommands() {
        // Check for clear all alerts command
        if (SmartDashboard.getBoolean("CAN/Pit/Commands/ClearAll", false)) {
            clearAllAlerts();
            SmartDashboard.putBoolean("CAN/Pit/Commands/ClearAll", false); // Reset button
            Logger.recordOutput("CANHealth/Commands/ClearAllTriggered", 
                RobotController.getFPGATime() / 1e6);
        }
        
        // Check for individual device alert clearing
        for (String deviceKey : new HashSet<>(activeAlerts.keySet())) {
            String commandKey = "CAN/Pit/Commands/Clear_" + deviceKey.replace("/", "_");
            if (SmartDashboard.getBoolean(commandKey, false)) {
                clearAlert(deviceKey);
                SmartDashboard.putBoolean(commandKey, false); // Reset button
                Logger.recordOutput("CANHealth/Commands/ClearDevice", deviceKey);
            }
        }
        
        // Publish available clear commands for dashboard to create buttons
        Set<String> clearableDevices = activeAlerts.entrySet().stream()
            .filter(entry -> !entry.getValue().cleared)
            .map(entry -> entry.getKey().replace("/", "_"))
            .collect(java.util.stream.Collectors.toSet());
        
        SmartDashboard.putStringArray("CAN/Pit/Commands/ClearableDevices", 
            clearableDevices.toArray(new String[0]));
    }

    /**
     * Periodic method to be called from Robot.java to update logging and dashboard.
     * This handles any maintenance tasks that need to run regularly.
     */
    public void periodic() {
        // Log overall health metrics
        AlertLevel severity = getCurrentMaxSeverity();
        Logger.recordOutput("CANHealth/OverallSeverity", severity.name());
        Logger.recordOutput("CANHealth/ActiveAlertCount", getAllActiveAlerts().size());
        
        // Log device connection counts by bus
        for (Map.Entry<String, List<String>> bus : wiringOrder.entrySet()) {
            int connected = 0;
            int total = bus.getValue().size();
            
            for (String device : bus.getValue()) {
                if (currentDeviceStatus.getOrDefault(device, false)) {
                    connected++;
                }
            }
            
            Logger.recordOutput("CANHealth/Bus_" + bus.getKey() + "/Connected", connected);
            Logger.recordOutput("CANHealth/Bus_" + bus.getKey() + "/Total", total);
        }
        
        updateDashboard();
    }

    /**
     * Returns diagnostic information about a specific CAN bus.
     * This can be used for advanced troubleshooting.
     */
    public String getBusStatus(String busName) {
        List<String> devices = wiringOrder.get(busName);
        if (devices == null) {
            return "Unknown bus: " + busName;
        }
        
        StringBuilder status = new StringBuilder();
        status.append("Bus: ").append(busName).append("\n");
        
        int connected = 0;
        for (String device : devices) {
            boolean isConnected = currentDeviceStatus.getOrDefault(device, false);
            status.append("  ").append(device).append(": ")
                  .append(isConnected ? "OK" : "DISCONNECTED").append("\n");
            if (isConnected) connected++;
        }
        
        status.append("Total: ").append(connected).append("/").append(devices.size())
              .append(" connected");
        
        return status.toString();
    }
}