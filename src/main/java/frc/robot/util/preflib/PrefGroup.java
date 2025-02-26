package frc.robot.util.preflib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

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

  public PrefGroup subgroup(String... keys) {
    if (keys.length == 0) {
      return this;
    }

    PrefGroup childGroup;
    if (subGroups.containsKey(keys[0])) {
      childGroup = subGroups.get(keys[0]);
    } else {
      childGroup = new PrefGroup(this.key + keys[0] + "/");
      subGroups.put(keys[0], childGroup);
    }

    return childGroup.subgroup(Arrays.copyOfRange(keys, 1, keys.length));
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
        throw new IllegalArgumentException(
            "Preference with name " + name + " already exists with a different type");
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

  public void createCurrentLimitPrefs(Consumer<CurrentLimitsConfigs> configuratorApply) {
    PrefGroup currentLimitsGroup = subgroup("CurrentLimits");
    class DefaultConfig extends CurrentLimitsConfigs {
      public DefaultConfig() {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.StatorCurrentLimit = 40;
        config.SupplyCurrentLimit = 40;
        config.SupplyCurrentLowerLimit = 40;
        config.SupplyCurrentLowerTime = 1;
        config.SupplyCurrentLimitEnable = false;
        config.StatorCurrentLimitEnable = true;
      }
    }
    Supplier<CurrentLimitsConfigs> defaultConfig =
        () -> {
          CurrentLimitsConfigs config = new CurrentLimitsConfigs();
          config.StatorCurrentLimit = 40;
          config.SupplyCurrentLimit = 40;
          config.SupplyCurrentLowerLimit = 40;
          config.SupplyCurrentLowerTime = 1;
          config.SupplyCurrentLimitEnable = false;
          config.StatorCurrentLimitEnable = true;
          return config;
        };
    currentLimitsGroup.applyAndListenToConfig(
        "StatorCurrentLimit", defaultConfig, configuratorApply, CurrentLimitsConfigs.class);
    currentLimitsGroup.applyAndListenToConfig(
        "SupplyCurrentLimit", defaultConfig, configuratorApply, CurrentLimitsConfigs.class);
    currentLimitsGroup.applyAndListenToConfig(
        "SupplyCurrentLowerLimit", defaultConfig, configuratorApply, CurrentLimitsConfigs.class);
    currentLimitsGroup.applyAndListenToConfig(
        "SupplyCurrentLowerTime", defaultConfig, configuratorApply, CurrentLimitsConfigs.class);
    currentLimitsGroup.applyAndListenToConfig(
        "SupplyCurrentLimitEnable", defaultConfig, configuratorApply, CurrentLimitsConfigs.class);
    currentLimitsGroup.applyAndListenToConfig(
        "StatorCurrentLimitEnable", defaultConfig, configuratorApply, CurrentLimitsConfigs.class);
  }

  protected static PrefGroup create(String... keys) {
    if (keys.length == 0) {
      return BasePrefGroup.INSTANCE;
    }

    return BasePrefGroup.INSTANCE.subgroup(keys);
  }

  protected void update() {
    for (Pref<?> pref : prefs.values()) {
      pref.update(false);
    }
    for (PrefGroup group : subGroups.values()) {
      group.update();
    }
  }

  private PrefGroup(String... keys) {
    this.key = String.join("/", keys);
  }

  private <ConfigType, ValueType> void applyAndListenToConfig(
      String name,
      Supplier<ConfigType> configGenerator,
      Consumer<ConfigType> apply,
      Class<ValueType> valueClass) {
    try {
      Field valueField = valueClass.getField(name);

      Consumer<ValueType> listener =
          (value) -> {
            ConfigType config = configGenerator.get();
            try {
              valueField.set(config, value);
            } catch (IllegalAccessException e) {
              System.err.println("Internal preflib error with " + name);
              e.printStackTrace();
            }
            apply.accept(config);
          };

      @SuppressWarnings("unchecked")
      ValueType defaultValue = (ValueType) valueField.get(configGenerator.get());

      applyAndListen(name, defaultValue, listener);
    } catch (NoSuchFieldException e) {
      System.err.println("Internal preflib error with " + name);
      e.printStackTrace();
    } catch (IllegalAccessException e) {
      System.err.println("Internal preflib error with " + name);
      e.printStackTrace();
    }
  }
}
