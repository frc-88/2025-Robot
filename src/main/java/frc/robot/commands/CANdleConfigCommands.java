package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Lights;

public class CANdleConfigCommands {
  public static class ConfigBrightness extends InstantCommand {
    public ConfigBrightness(Lights candleSystem, double brightnessPercent) {
      super(() -> candleSystem.configBrightness(brightnessPercent), candleSystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public static class ConfigLosBehavior extends InstantCommand {
    public ConfigLosBehavior(Lights candleSystem, boolean disableWhenLos) {
      super(() -> candleSystem.configLos(disableWhenLos), candleSystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public static class ConfigStatusLedBehavior extends InstantCommand {
    public ConfigStatusLedBehavior(Lights candleSystem, boolean disableWhile) {
      super(() -> candleSystem.configStatusLedBehavior(disableWhile), candleSystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }
}
