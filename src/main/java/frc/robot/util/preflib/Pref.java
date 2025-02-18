package frc.robot.util.preflib;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Preferences;

public class Pref<T> {
    
    private final String key;
    private final T defaultValue;
    private T currentValue;
    private final List<Consumer<T>> listeners = new LinkedList<>();
    private static final List<Pref<?>> prefs = new LinkedList<>();

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

    public void addChangeListener(Consumer<T> listener) {
        listeners.add(listener);
    }

    public static void updateAll() {
        for (Pref<?> pref : prefs) {
            pref.update(false);
        }
    }

    protected Pref(String key, T defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;

        if (!Preferences.containsKey(key)) {
            init(defaultValue);
            this.currentValue = defaultValue;
        } else {
            update(false);
        }
    }

    @SuppressWarnings("unchecked")
    protected void update(boolean force) {
        T newValue;

        if (defaultValue instanceof Integer) {
            newValue = (T) Integer.valueOf(Preferences.getInt(key, (Integer) defaultValue));
        } else if (defaultValue instanceof Double) {
            newValue = (T) Double.valueOf(Preferences.getDouble(key, (Double) defaultValue));
        } else if (defaultValue instanceof Boolean) {
            newValue = (T) Boolean.valueOf(Preferences.getBoolean(key, (Boolean) defaultValue));
        } else if (defaultValue instanceof String) {
            newValue = (T) Preferences.getString(key, (String) defaultValue);
        } else if (defaultValue instanceof Enum) {
            String enumValue = Preferences.getString(key, ((Enum<?>) defaultValue).name());
            newValue = (T) Enum.valueOf(((Enum<?>) defaultValue).getDeclaringClass(), enumValue);
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
