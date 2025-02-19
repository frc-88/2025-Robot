package frc.robot.util.preflib;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Preferences;

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

    protected Class<?> getType() {
        return currentValue.getClass();
    }

    @SuppressWarnings("unchecked")
    protected void update(boolean force) {
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

        if (force || !newValue.equals(currentValue)) {
            currentValue = newValue;
            for (Consumer<?> listener : listeners) {
                ((Consumer<T>) listener).accept(newValue);
            }
        }
    }

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
