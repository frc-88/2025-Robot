package frc.robot.util.preflib;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class PrefGroup {

    private static class BasePrefGroup extends PrefGroup {
        private static final BasePrefGroup INSTANCE = new BasePrefGroup();
        
        BasePrefGroup() {
            super();
        }
    }

    private final String key;
    private final Map<String, PrefGroup> subGroups = new HashMap<>();
    private final Map<String, Pref<?>> prefs = new HashMap<>();

    public PrefGroup subGroup(String... keys) {
        if (keys.length == 0) {
            return this;
        }

        PrefGroup childGroup;
        if (subGroups.containsKey(keys[0])) {
            childGroup = subGroups.get(keys[0]);
        } else {
            childGroup = new PrefGroup(this.key + "/" + keys[0]);
            subGroups.put(keys[0], childGroup);
        }

        return childGroup.subGroup(Arrays.copyOfRange(keys, 1, keys.length));
    }


    public <T> Pref<T> getPref(String name, T defaultValue) {
        if (subGroups.containsKey(name)) {
            throw new IllegalArgumentException("Name " + name + " is already used as a subgroup");
        }

        if (prefs.containsKey(name)) {
            Pref<?> pref = prefs.get(name);
            if (pref.getType().equals(defaultValue.getClass())) {
                @SuppressWarnings("unchecked")
                Pref<T> castedPref = (Pref<T>) pref;
                return castedPref;
            } else {
                throw new IllegalArgumentException("Preference with name " + name + " already exists with a different type");
            }
        }

        Pref<T> pref = new Pref<T>(key + name, defaultValue);
        prefs.put(name, pref);
        return pref;
    }

    public <T> T getValue(String name, T defaultValue) {
        return getPref(name, defaultValue).get();
    }

    public <T> void applyAndListen(String name, T defaultValue, Consumer<T> listener) {
        getPref(name, defaultValue).applyAndListen(listener);
    }

    protected static PrefGroup create(String... keys) {
        if (keys.length == 0) {
            return BasePrefGroup.INSTANCE;
        }

        return BasePrefGroup.INSTANCE.subGroup(keys);
    }

    protected void update() {
        for (Pref<?> pref : prefs.values()) {
            pref.update(true);
        }
        for (PrefGroup group : subGroups.values()) {
            group.update();
        }
    }

    private PrefGroup(String... keys) {
        this.key = String.join("/", keys);
    }
}
