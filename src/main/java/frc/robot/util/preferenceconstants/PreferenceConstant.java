package frc.robot.util.preferenceconstants;

import java.util.function.Consumer;

public interface PreferenceConstant<T> extends UpdatableConstant {

  /** Reads in the value from WPILib preferences, notifying the change handler if necessary. */
  @Override
  public void update();

  /**
   * Gets the value that was read on the last update of this constant.
   *
   * @return The value
   */
  public T getValue();

  /** Assigns a change handler for this constant that will get called whenever the value changes. */
  public void addChangeHandler(Consumer<T> handler);

  /**
   * Sets the value of this constant, update WPILib preferences and calling the change handler.
   *
   * @param value The value to set.
   */
  public void setValue(T value);

  /**
   * Initializes the value of this constant, update WPILib preferences and calling the change
   * handler.
   *
   * @param value The value to set.
   */
  public void initValue(T value);
}
