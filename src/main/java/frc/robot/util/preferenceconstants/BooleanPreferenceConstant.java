package frc.robot.util.preferenceconstants;

import edu.wpi.first.wpilibj.Preferences;
import java.util.Objects;

/** Preferences constant for boolean values. */
public class BooleanPreferenceConstant extends BasePreferenceConstant<Boolean> {

  String name;
  boolean defaultValue;

  /**
   * Constructor. Will call update() once.
   *
   * @param name The name to be used as a key in WPILib preferences
   * @param defaultValue The value that will be set as default if the value doesn't exist in WPILib
   *     preferences
   */
  public BooleanPreferenceConstant(String name, boolean defaultValue) {
    this.name = Objects.requireNonNull(name);
    this.defaultValue = Objects.requireNonNull(defaultValue);
    if (!Preferences.containsKey(name)) {
      this.initValue(defaultValue);
    } else {
      update();
    }
  }

  @Override
  protected Boolean getFromPreferences() {
    return Preferences.getBoolean(name, defaultValue);
  }

  @Override
  protected void setInPreferences(Boolean value) {
    Preferences.setBoolean(name, value);
  }

  @Override
  protected void initInPreferences(Boolean value) {
    Preferences.initBoolean(name, value);
  }
}
