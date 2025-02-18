package frc.robot.util.preflib;

import java.util.function.Consumer;

public class PrefGroup {

    private final String key;

    public PrefGroup(String... keys) {
        this.key = String.join("/", keys) + "/";
    }

    public PrefGroup subGroup(String... keys) {
        return new PrefGroup(this.key + String.join("/", keys));
    }

    public <T> Pref<T> getPref(String name, T defaultValue) {
        return new Pref<T>(key + name, defaultValue);
    }

    public <T> T getValue(String name, T defaultValue) {
        return getPref(name, defaultValue).get();
    }

    public <T> void applyAndListen(String name, Consumer<T> listener) {
        Pref<T> pref = getPref(name, null);
        pref.addChangeListener(listener);
        pref.update(true);
    }
}
