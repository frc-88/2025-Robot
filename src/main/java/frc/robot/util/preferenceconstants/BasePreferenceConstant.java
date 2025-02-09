package frc.robot.util.preferenceconstants;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

/** Base class of the PreferenceConstant interface. */
public abstract class BasePreferenceConstant<T> implements PreferenceConstant<T> {

  private T value;
  private List<Consumer<T>> changeHandlers;

  public BasePreferenceConstant() {
    PreferenceConstants.addUpdatableConstant(this);
    changeHandlers = new LinkedList<>();
  }

  /**
   * Get the value of this constant from WPILib Preferences.
   *
   * @return The value from Preferences
   */
  protected abstract T getFromPreferences();

  /**
   * Set the value of this constant in WPILib Preferences.
   *
   * @param value The value to set
   */
  protected abstract void setInPreferences(T value);

  /**
   * Init the value of this constant in WPILib Preferences.
   *
   * @param value The value to set
   */
  protected abstract void initInPreferences(T value);

  @Override
  public final void update() {
    T newValue = this.getFromPreferences();
    boolean changed = !newValue.equals(this.value);
    this.value = newValue;
    if (changed) {
      for (Consumer<T> handler : this.changeHandlers) {
        handler.accept(newValue);
      }
    }
  }

  @Override
  public T getValue() {
    return this.value;
  }

  @Override
  public void addChangeHandler(Consumer<T> handler) {
    this.changeHandlers.add(handler);
  }

  @Override
  public final void setValue(T value) {
    this.setInPreferences(value);
    this.update();
  }

  @Override
  public final void initValue(T value) {
    this.initInPreferences(value);
    this.update();
  }
}
