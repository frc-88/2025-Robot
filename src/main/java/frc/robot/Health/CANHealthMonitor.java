package frc.robot.Health;

import java.util.List;
import java.util.Map;

/**
 * Central CAN health monitor for collecting device statuses and analyzing CAN bus health.
 *
 * <p>Next steps: - Add deviceStatus storage and updateStatus() method - Implement diagnostic
 * utilities: getBusStatus(), findBusBreakpoint(), etc.
 *
 * <p>To define your physical CAN wiring topology, populate the wiringOrder map below. Each entry in
 * the List must exactly match a key used in updateStatus calls, and the list should reflect the
 * true, daisy-chain order of devices on that bus.
 */
public class CANHealthMonitor {
  // Singleton instance
  private static CANHealthMonitor instance;

  /**
   * Defines the physical wiring order for each CAN bus.
   *
   * <p><busName> : List of device-status keys in the order they appear on the trunk. Device keys
   * must match the strings passed to updateStatus(), e.g.: "Drive/FrontLeft/DriveMotor"
   * "Drive/FrontLeft/TurnMotor" ...
   */
  private final Map<String, List<String>> wiringOrder =
      Map.of(
          // Primary roboRIO CAN bus (first trunk segment)
          /* Use this list if you have a second CAN bus
          "CANivore", List.of(
              // TODO: replace these examples with your actual device keys in wiring order:
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
              // ... more devices on 'CANivore' bus ...
          ),
          */
          // Secondary CAN bus (e.g., on a CANivore or second trunk)
          "Rio",
          List.of(
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
              // ... more devices on 'rio' bus ...
              ));

  // Private constructor to enforce singleton pattern
  private CANHealthMonitor() {}

  /** Returns the single instance of the health monitor. */
  public static CANHealthMonitor getInstance() {
    if (instance == null) {
      instance = new CANHealthMonitor();
    }
    return instance;
  }

  // TODO: Add:
  //    - private Map<String, Boolean> deviceStatus;
  //    - public void updateStatus(String key, boolean isReady)
  //    - public Map<String, Boolean> getAllStatuses()
  //    - diagnostics: getBusStatus(String busName), findBusBreakpoint(String busName)
}
