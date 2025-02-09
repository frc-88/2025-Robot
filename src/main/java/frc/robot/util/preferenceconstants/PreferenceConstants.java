package frc.robot.util.preferenceconstants;

import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

/** This class holds all of the constants for this project. */
public class PreferenceConstants {

  /** Updates all UpdatableConstants. */
  public static void update() {
    if (Objects.isNull(updatableConstants)) {
      return;
    }

    for (UpdatableConstant constant : updatableConstants) {
      constant.update();
    }
  }

  // The updatable constants registered.
  private static List<UpdatableConstant> updatableConstants;

  /**
   * Registers an updatable constant to be updated.
   *
   * @param constant The constant to be registered
   */
  protected static void addUpdatableConstant(UpdatableConstant constant) {
    if (Objects.isNull(updatableConstants)) {
      updatableConstants = new LinkedList<>();
    }
    updatableConstants.add(constant);
  }
}
