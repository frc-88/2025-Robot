package frc.robot.util.preflib;

import edu.wpi.first.wpilibj.Preferences;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Pref<T> {

  private final String key;
  private T currentValue;
  private final List<Consumer<T>> listeners = new LinkedList<>();

  public void set(T value) {
    if (value instanceof Integer) {
      Preferences.setInt(key, (Integer) value);
    } else if (value instanceof Double) {
      Preferences.setDouble(key, (Double) value);
    } else if (value instanceof Boolean) {
      Preferences.setBoolean(key, (Boolean) value);
    } else if (value instanceof String) {
      Preferences.setString(key, (String) value);
    } else if (value instanceof Enum) {
      Preferences.setString(key, ((Enum<?>) value).name());
    } else {
      throw new IllegalArgumentException("Unsupported preference type");
    }
    update(false);
  }

  public T get() {
    return currentValue;
  }

  public void applyAndListen(Consumer<T> listener) {
    addChangeListener(listener);
    update(true);
  }

  public void addChangeListener(Consumer<T> listener) {
    listeners.add(listener);
  }

  protected Pref(String key, T defaultValue) {
    this.key = key;
    this.currentValue = defaultValue;

    if (!Preferences.containsKey(key)) {
      init(defaultValue);
    } else {
      update(false);
    }
  }

  protected Pref(String key) {
    this.key = key;
  }

  protected Class<?> getType() {
    return currentValue.getClass();
  }

  @SuppressWarnings("unchecked")
  protected void update(boolean force) {
    if (!Logger.hasReplaySource()) {
      T newValue;

      if (currentValue instanceof Integer) {
        newValue = (T) Integer.valueOf(Preferences.getInt(key, (Integer) currentValue));
      } else if (currentValue instanceof Double) {
        newValue = (T) Double.valueOf(Preferences.getDouble(key, (Double) currentValue));
      } else if (currentValue instanceof Boolean) {
        newValue = (T) Boolean.valueOf(Preferences.getBoolean(key, (Boolean) currentValue));
      } else if (currentValue instanceof String) {
        newValue = (T) Preferences.getString(key, (String) currentValue);
      } else if (currentValue instanceof Enum) {
        String enumValue = Preferences.getString(key, ((Enum<?>) currentValue).name());
        newValue = (T) Enum.valueOf(((Enum<?>) currentValue).getDeclaringClass(), enumValue);
      } else {
        throw new IllegalArgumentException("Unsupported preference type");
      }

      if (force
          || (newValue instanceof Double
              && Math.abs((Double) newValue - (Double) currentValue) > 0.001)
          || (!(newValue instanceof Double) && !newValue.equals(currentValue))) {
        currentValue = newValue;
        for (Consumer<?> listener : listeners) {
          ((Consumer<T>) listener).accept(newValue);
        }
      }
    }

    Logger.processInputs("NetworkInputs", inputs);
  }

  private final LoggableInputs inputs =
      new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
          // Casting is necessary because put is an overloaded method which needs to know the type
          // passed as the second value.
          if (currentValue instanceof Integer) {
            table.put("Tuning/" + key, (Integer) currentValue);
          } else if (currentValue instanceof Double) {
            table.put("Tuning/" + key, (Double) currentValue);
          } else if (currentValue instanceof Boolean) {
            table.put("Tuning/" + key, (Boolean) currentValue);
          } else if (currentValue instanceof String) {
            table.put("Tuning/" + key, (String) currentValue);
          } else if (currentValue instanceof Enum) {
            table.put("Tuning/" + key, ((Enum<?>) currentValue).name());
          }
        }

        @Override
        @SuppressWarnings("unchecked")
        public void fromLog(LogTable table) {
          if (currentValue instanceof Integer) {
            currentValue = (T) Integer.valueOf(table.get("Tuning/" + key, (Integer) currentValue));
          } else if (currentValue instanceof Double) {
            currentValue = (T) Double.valueOf(table.get("Tuning/" + key, (Double) currentValue));
          } else if (currentValue instanceof Boolean) {
            currentValue = (T) Boolean.valueOf(table.get("Tuning/" + key, (Boolean) currentValue));
          } else if (currentValue instanceof String) {
            currentValue = (T) table.get("Tuning/" + key, (String) currentValue);
          } else if (currentValue instanceof Enum) {
            String enumValue = table.get("Tuning/" + key, ((Enum<?>) currentValue).name());
            currentValue =
                (T) Enum.valueOf(((Enum<?>) currentValue).getDeclaringClass(), enumValue);
          }
        }
      };

  private void init(T value) {
    if (value instanceof Integer) {
      Preferences.initInt(key, (Integer) value);
    } else if (value instanceof Double) {
      Preferences.initDouble(key, (Double) value);
    } else if (value instanceof Boolean) {
      Preferences.initBoolean(key, (Boolean) value);
    } else if (value instanceof String) {
      Preferences.initString(key, (String) value);
    } else if (value instanceof Enum) {
      Preferences.initString(key, ((Enum<?>) value).name());
    } else {
      throw new IllegalArgumentException("Unsupported preference type");
    }
  }
}
