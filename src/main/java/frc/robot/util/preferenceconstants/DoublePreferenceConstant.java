package frc.robot.util.preferenceconstants;

import edu.wpi.first.wpilibj.Preferences;
import java.util.Objects;

/** Preferences constant for double values */
public class DoublePreferenceConstant extends BasePreferenceConstant<Double> {

  String name;
  double defaultValue;

  /**
   * Constructor. Will call update() once.
   *
   * @param name The name to be used as a key in WPILib preferences
   * @param defaultValue The value that will be set as default if the value doesn't exist in WPILib
   *     preferences
   */
  public DoublePreferenceConstant(String name, double defaultValue) {
    this.name = Objects.requireNonNull(name);
    this.defaultValue = Objects.requireNonNull(defaultValue);
    if (!Preferences.containsKey(name)) {
      this.initValue(defaultValue);
    } else {
      update();
    }
  }

  @Override
  protected Double getFromPreferences() {
    return Preferences.getDouble(name, defaultValue);
  }

  @Override
  protected void setInPreferences(Double value) {
    Preferences.setDouble(name, value);
  }

  @Override
  protected void initInPreferences(Double value) {
    Preferences.initDouble(name, value);
  }
}
