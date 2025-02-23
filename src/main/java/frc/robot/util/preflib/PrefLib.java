package frc.robot.util.preflib;

public class PrefLib {
  private static PrefGroup rootGroup = PrefGroup.create();

  public static PrefGroup getGroup(String... keys) {
    return rootGroup.subgroup(keys);
  }

  public static void update() {
    rootGroup.update();
  }
}
