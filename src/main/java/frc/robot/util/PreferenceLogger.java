package frc.robot.util;  // <-- adjust to match your project

import java.util.Collection;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;

import org.littletonrobotics.junction.Logger;

/**
 * Utility for taking one-time snapshots of ALL WPILib Preferences
 * and recording them into the AdvantageKit log. Handles double, boolean,
 * string, and numeric types automatically.
 * 
 * Usage example (in Robot.java):
 *
 *   @Override
 *   public void autonomousInit() {
 *     PreferenceLogger.logAllPreferences("autoInit");
 *     // ... existing code ...
 *   }
 *
 *   @Override
 *   public void teleopInit() {
 *     PreferenceLogger.logAllPreferences("teleopInit");
 *     // ... existing code ...
 *   }
 *
 * This will create log entries like:
 *   Preferences/autoInit/ShooterKp
 *   Preferences/teleopInit/ShooterKp
 *
 * Each entry gets a single sample at the moment the snapshot is taken.
 */
public final class PreferenceLogger {

  private PreferenceLogger() {}

  /**
   * Logs a one-time snapshot of all current WPILib Preferences into AdvantageKit.
   *
   * Creates outputs like:
   *   Preferences/<label>/<key>
   *
   * @param label A short name such as "autoInit" or "teleopInit".
   */
  public static void logAllPreferences(String label) {

    // Access the NetworkTables backing store for Preferences
    NetworkTable prefsTable =
        NetworkTableInstance.getDefault().getTable("Preferences");

    // Keys stored in the Preferences API
    Collection<String> keys = Preferences.getKeys();

    for (String key : keys) {

      NetworkTableEntry entry = prefsTable.getEntry(key);
      NetworkTableType type = entry.getType();

      String baseName = "Preferences/" + label + "/" + key;

      switch (type) {

        case kDouble:
          Logger.recordOutput(baseName, entry.getDouble(Double.NaN));
          break;

        case kBoolean:
          Logger.recordOutput(baseName, entry.getBoolean(false));
          break;

        case kString:
          Logger.recordOutput(baseName, entry.getString(""));
          break;

        case kInteger: // int64
          Logger.recordOutput(baseName, entry.getInteger(0));
          break;

        // Numeric arrays or string arrays are rare in Preferences,
        // but we can support them for completeness:
        case kDoubleArray:
          Logger.recordOutput(baseName, entry.getDoubleArray(new double[0]));
          break;

        case kBooleanArray:
          Logger.recordOutput(baseName, entry.getBooleanArray(new boolean[0]));
          break;

        case kStringArray:
          Logger.recordOutput(baseName, entry.getStringArray(new String[0]));
          break;

        // If unexpected or unhandled, record nothing (or put
        // a placeholder if you want)
        default:
          // Optional: Logger.recordOutput(baseName, Double.NaN);
          break;
      }
    }
  }
}
