package frc.robot.health;

import java.util.Map;
import java.util.List;

/**
 * Central CAN health monitor for collecting device statuses and analyzing CAN bus health.
 *\n * Next steps: add deviceStatus map, status-update methods, and diagnostic utilities.
 */
public class CANHealthMonitor {
    // Singleton instance
    private static CANHealthMonitor instance;

    /**
     * Defines the physical wiring order for each CAN bus.
     * Keys are bus names (as used in device constructors), values are the ordered list of
     * device-status keys (matching those used in updateStatus calls).
     */
    private final Map<String, List<String>> wiringOrder = Map.of(
        "rio", List.of(
            // TODO: Populate with device keys in physical order, e.g. "Drive/FrontLeft/DriveMotor"
        ),
        "CANivore", List.of(
            // TODO: Populate with device keys for second CAN bus
        )
    );

    // Private constructor to enforce singleton pattern
    private CANHealthMonitor() {
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

    // TODO: Add deviceStatus storage, updateStatus(), and diagnostic methods
}
