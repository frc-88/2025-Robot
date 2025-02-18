package frc.robot.util.preferenceconstants;

import java.util.function.Consumer;

/** Class used for MotionMagic control, using Velocity, Acceleration, Jerk, and PID control */
public class MotionMagicPreferenceConstants {

  private DoublePreferenceConstant maxVelocity;
  private DoublePreferenceConstant maxAcceleration;
  private DoublePreferenceConstant maxJerk;

  private DoublePreferenceConstant kP;
  private DoublePreferenceConstant kI;
  private DoublePreferenceConstant kD;
  private DoublePreferenceConstant kV;
  private DoublePreferenceConstant kS;

  /**
   * Consturctor for MotionMagic PreferenceConstants
   *
   * @param name The name of the Preference Constants
   * @param maxVelocity The Max Velocity the Motion Profile can reach. Units should be rotations per
   *     second
   * @param maxAcceleration The Max Acceleration the Motion Profile can reach. Units should be
   *     rotations per second squared
   * @param maxJerk The Max Jerk the Motion Profile can reach. Units should be rotations per second
   *     cubed
   * @param kP The proportional gain
   * @param kI The integral gain
   * @param kD The derivative gain
   * @param kV The velocity feedforward gain. Units should be determined depending on the request
   *     used
   * @param kS The static friction feedforward gain. Units should be determined depending on the
   *     request used
   */
  public MotionMagicPreferenceConstants(
      String name,
      double maxVelocity,
      double maxAcceleration,
      double maxJerk,
      double kP,
      double kI,
      double kD,
      double kV,
      double kS) {
    this.maxVelocity =
        new DoublePreferenceConstant(String.format("%s MaxVelocity", name), maxVelocity);
    this.maxAcceleration =
        new DoublePreferenceConstant(String.format("%s MaxAcceleration", name), maxAcceleration);
    this.maxJerk = new DoublePreferenceConstant(String.format("%s MaxJerk", name), maxJerk);
    this.kP = new DoublePreferenceConstant(String.format("%s kP", name), kP);
    this.kI = new DoublePreferenceConstant(String.format("%s kI", name), kI);
    this.kD = new DoublePreferenceConstant(String.format("%s kD", name), kD);
    this.kV = new DoublePreferenceConstant(String.format("%s kV", name), kV);
    this.kS = new DoublePreferenceConstant(String.format("%s kS", name), kS);
  }

  public MotionMagicPreferenceConstants(String name) {
    this(name, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  /**
   * Get the MaxVelocity PreferenceConstant
   *
   * @return The MaxVelocity PreferenceConstant
   */
  public DoublePreferenceConstant getMaxVelocity() {
    return maxVelocity;
  }

  /**
   * Get the MaxAcceleration PreferenceConstant
   *
   * @return The MaxAcceleration PreferenceConstant
   */
  public DoublePreferenceConstant getMaxAcceleration() {
    return maxAcceleration;
  }

  /**
   * Get the MaxJerk PreferenceConstant
   *
   * @return The MaxJerk PreferenceConstant
   */
  public DoublePreferenceConstant getMaxJerk() {
    return maxJerk;
  }

  /**
   * Get the kP PreferenceConstant
   *
   * @return The kP PreferenceConstant
   */
  public DoublePreferenceConstant getKP() {
    return kP;
  }

  /**
   * Get the kI PreferenceConstant
   *
   * @return The kI PreferenceConstant
   */
  public DoublePreferenceConstant getKI() {
    return kI;
  }

  /**
   * Get the kD PreferenceConstant
   *
   * @return The kD PreferenceConstant
   */
  public DoublePreferenceConstant getKD() {
    return kD;
  }

  /**
   * Get the kV PreferenceConstant
   *
   * @return The kV PreferenceConstant
   */
  public DoublePreferenceConstant getKV() {
    return kV;
  }

  /**
   * Get the kS PreferenceConstant
   *
   * @return The kS PreferenceConstant
   */
  public DoublePreferenceConstant getKS() {
    return kS;
  }

  /** Update Velocity, Acceleration, Jerk, and PID PreferenceConstants */
  public void updateAll() {
    this.maxVelocity.update();
    this.maxAcceleration.update();
    this.maxJerk.update();
    this.kP.update();
    this.kI.update();
    this.kD.update();
    this.kV.update();
    this.kS.update();
  }

  /**
   * Add the Consumer responsible taking in the preference and updating
   *
   * @param handler The Consumer for updating the constant
   */
  public void addChangeHandler(Consumer<Double> handler) {
    this.maxVelocity.addChangeHandler(handler);
    this.maxAcceleration.addChangeHandler(handler);
    this.maxJerk.addChangeHandler(handler);
    this.kP.addChangeHandler(handler);
    this.kI.addChangeHandler(handler);
    this.kD.addChangeHandler(handler);
    this.kV.addChangeHandler(handler);
    this.kS.addChangeHandler(handler);
  }
}
