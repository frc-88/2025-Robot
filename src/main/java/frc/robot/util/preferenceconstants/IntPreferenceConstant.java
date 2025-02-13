package frc.robot.util.preferenceconstants;

import edu.wpi.first.wpilibj.Preferences;
import java.util.Objects;

/** Preferences constant for int values. */
public class IntPreferenceConstant extends BasePreferenceConstant<Integer> {

  String name;
  int defaultValue;

  /**
   * Constructor. Will call update() once.
   *
   * @param name The name to be used as a key in WPILib preferences
   * @param defaultValue The value that will be set as default if the value doesn't exist in WPILib
   *     preferences
   */
  public IntPreferenceConstant(String name, int defaultValue) {
    this.name = Objects.requireNonNull(name);
    this.defaultValue = Objects.requireNonNull(defaultValue);
    if (!Preferences.containsKey(name)) {
      this.initValue(defaultValue);
    } else {
      update();
    }
  }

  @Override
  protected Integer getFromPreferences() {
    return Preferences.getInt(name, defaultValue);
  }

  @Override
  protected void setInPreferences(Integer value) {
    Preferences.setInt(name, value);
  }

  @Override
  protected void initInPreferences(Integer value) {
    Preferences.initInt(name, value);
  }
}
