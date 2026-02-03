package frc.robot.Health;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;

/**
 * Central CAN health monitor for collecting device statuses and analyzing CAN bus health.
 *
 * <p>This singleton class provides real-time monitoring of CAN device connectivity for DEV1 base,
 * which includes swerve drive modules, gyro, and vision cameras.
 *
 * <p>Phase 1 Implementation: Basic device status tracking with AdvantageKit logging integration.
 */
public class CANHealthMonitor {
  // Singleton instance
  private static CANHealthMonitor instance;

  // Phase 1: Simple device status tracking
  private final Map<String, Boolean> currentDeviceStatus = new ConcurrentHashMap<>();
  private final Map<String, Queue<StatusEvent>> deviceEvents = new ConcurrentHashMap<>();

  // Event history duration (10 seconds for short-term memory)
  private static final double EVENT_HISTORY_DURATION = 10.0;

  /**
   * Defines the physical wiring order for each CAN bus on DEV1. Device keys must match the strings
   * passed to updateStatus().
   *
   * <p>IMPORTANT: Update these lists to match your actual DEV1 CAN bus configuration!
   */
  private final Map<String, List<String>> wiringOrder =
      Map.of(
          "rio",
              List.of(
                  // Drive system devices - update based on your actual CAN IDs and bus
                  // configuration
                  "Drive/FrontLeft/DriveMotor",
                  "Drive/FrontLeft/TurnMotor",
                  "Drive/FrontLeft/TurnEncoder",
                  "Drive/FrontRight/DriveMotor",
                  "Drive/FrontRight/TurnMotor",
                  "Drive/FrontRight/TurnEncoder",
                  "Drive/BackLeft/DriveMotor",
                  "Drive/BackLeft/TurnMotor",
                  "Drive/BackLeft/TurnEncoder",
                  "Drive/BackRight/DriveMotor",
                  "Drive/BackRight/TurnMotor",
                  "Drive/BackRight/TurnEncoder",
                  "Drive/Gyro"),
          "canivore",
              List.of(
                  // Add CANivore devices here if you're using a second CAN bus
                  // Leave empty if all devices are on the RIO CAN bus
                  ));

  /** Represents a status change event for a CAN device. */
  private static class StatusEvent {
    public final double timestamp;
    public final boolean connected;

    public StatusEvent(boolean connected) {
      this.timestamp = RobotController.getFPGATime() / 1e6; // Convert microseconds to seconds
      this.connected = connected;
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

  /** Returns the single instance of the health monitor. */
  public static CANHealthMonitor getInstance() {
    if (instance == null) {
      instance = new CANHealthMonitor();
    }
    return instance;
  }

  /**
   * Updates the connection status for a CAN device. This should be called from each subsystem's
   * periodic() method.
   *
   * @param deviceKey Unique identifier for the device (e.g., "Drive/FrontLeft/DriveMotor")
   * @param isConnected Current connection status of the device
   */
  public void updateStatus(String deviceKey, boolean isConnected) {
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

      // Log status change to AdvantageKit
      Logger.recordOutput("CANHealth/Events/" + deviceKey.replace("/", "_"), isConnected);
      Logger.recordOutput(
          "CANHealth/StatusChange/" + deviceKey.replace("/", "_") + "_timestamp",
          RobotController.getFPGATime() / 1e6);
    }

    // Always log current status for AdvantageKit
    Logger.recordOutput("CANHealth/CurrentStatus/" + deviceKey.replace("/", "_"), isConnected);
  }

  /**
   * Updates the connection status for devices that use Phoenix 6 hardware objects. Convenience
   * method for devices that have isConnected() methods.
   */
  public void updateStatus(String deviceKey, Object device) {
    if (device == null) {
      updateStatus(deviceKey, false);
      return;
    }

    // Try to call isConnected() via reflection for Phoenix devices
    try {
      java.lang.reflect.Method method = device.getClass().getMethod("isConnected");
      boolean isConnected = (boolean) method.invoke(device);
      updateStatus(deviceKey, isConnected);
    } catch (Exception e) {
      // If reflection fails, assume connected (device object exists)
      updateStatus(deviceKey, true);
    }
  }

  /** Removes old events from the queue to keep memory usage bounded. */
  private void cleanupOldEvents(Queue<StatusEvent> events) {
    double currentTime = RobotController.getFPGATime() / 1e6;
    while (!events.isEmpty() && (currentTime - events.peek().timestamp) > EVENT_HISTORY_DURATION) {
      events.poll();
    }
  }

  /** Returns current connection status for all monitored devices. */
  public Map<String, Boolean> getAllDeviceStatus() {
    return new HashMap<>(currentDeviceStatus);
  }

  /** Returns diagnostic information about a specific CAN bus. */
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
      status
          .append("  ")
          .append(device)
          .append(": ")
          .append(isConnected ? "OK" : "DISCONNECTED")
          .append("\n");
      if (isConnected) connected++;
    }

    status
        .append("Total: ")
        .append(connected)
        .append("/")
        .append(devices.size())
        .append(" connected");

    return status.toString();
  }

  /** Returns the number of connected devices on a specific bus. */
  public int getConnectedDeviceCount(String busName) {
    List<String> devices = wiringOrder.get(busName);
    if (devices == null) return 0;

    int connected = 0;
    for (String device : devices) {
      if (currentDeviceStatus.getOrDefault(device, false)) {
        connected++;
      }
    }
    return connected;
  }

  /** Returns the total number of devices on a specific bus. */
  public int getTotalDeviceCount(String busName) {
    List<String> devices = wiringOrder.get(busName);
    return devices != null ? devices.size() : 0;
  }

  /**
   * Periodic method to be called from Robot.java to update logging and dashboard. This should be
   * called from Robot.robotPeriodic().
   */
  public void periodic() {
    // Log device connection counts by bus
    for (Map.Entry<String, List<String>> bus : wiringOrder.entrySet()) {
      int connected = getConnectedDeviceCount(bus.getKey());
      int total = getTotalDeviceCount(bus.getKey());

      Logger.recordOutput("CANHealth/Bus_" + bus.getKey() + "/Connected", connected);
      Logger.recordOutput("CANHealth/Bus_" + bus.getKey() + "/Total", total);
    }

    // Log overall health metrics
    int totalConnected = currentDeviceStatus.values().stream().mapToInt(b -> b ? 1 : 0).sum();
    int totalDevices = currentDeviceStatus.size();

    Logger.recordOutput("CANHealth/Overall/Connected", totalConnected);
    Logger.recordOutput("CANHealth/Overall/Total", totalDevices);
    // Update SmartDashboard for basic driver display
    updateDashboard();
  }

  /** Updates SmartDashboard with current status for driver display. */
  private void updateDashboard() {
    int totalConnected = currentDeviceStatus.values().stream().mapToInt(b -> b ? 1 : 0).sum();
    int totalDevices = currentDeviceStatus.size();

    SmartDashboard.putNumber("CAN/ConnectedDevices", totalConnected);
    SmartDashboard.putNumber("CAN/TotalDevices", totalDevices);
    SmartDashboard.putBoolean("CAN/AllDevicesOK", totalConnected == totalDevices);

    // Detailed bus status
    for (String busName : wiringOrder.keySet()) {
      int connected = getConnectedDeviceCount(busName);
      int total = getTotalDeviceCount(busName);
      SmartDashboard.putString("CAN/" + busName + "_Status", connected + "/" + total);
    }
  }

  /** Returns a simple health status string for dashboard display. */
  public String getOverallStatus() {
    int totalConnected = currentDeviceStatus.values().stream().mapToInt(b -> b ? 1 : 0).sum();
    int totalDevices = currentDeviceStatus.size();

    if (totalDevices == 0) {
      return "No devices monitored";
    } else if (totalConnected == totalDevices) {
      return "All CAN devices OK";
    } else {
      return String.format("CAN issues: %d/%d devices connected", totalConnected, totalDevices);
    }
  }
}
