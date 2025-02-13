package frc.robot.util.preferenceconstants;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Collection of PreferenceConstants for gains and values that are typically used by PID. A
 * particular PID implementation may choose to ignore some of these values.
 */
public class PIDPreferenceConstants {

  private DoublePreferenceConstant kP;
  private DoublePreferenceConstant kI;
  private DoublePreferenceConstant kD;
  private DoublePreferenceConstant kF;
  private DoublePreferenceConstant iZone;
  private DoublePreferenceConstant iMax;
  private DoublePreferenceConstant tolerance;
  private DoublePreferenceConstant kS;

  /**
   * Constructor. All values given are defaults that will be overriden if a preference for them
   * already exists.
   *
   * @param name The name to pre-pend to all preference names
   * @param kP The proportional gain
   * @param kI The integral gain
   * @param kD The differential gain
   * @param kF The feedforward gain
   * @param kS The static friction value
   * @param iZone The error range in which the integral accumulates
   * @param iMax The max error absolute value that the integral will acumulate
   * @param tolerance The minimim error absolute value where an output will be applied
   */
  public PIDPreferenceConstants(
      String name,
      double kP,
      double kI,
      double kD,
      double kF,
      double kS,
      double iZone,
      double iMax,
      double tolerance) {
    Objects.requireNonNull(name);
    this.kP = new DoublePreferenceConstant(name + " kP", kP);
    this.kI = new DoublePreferenceConstant(name + " kI", kI);
    this.kD = new DoublePreferenceConstant(name + " kD", kD);
    this.kF = new DoublePreferenceConstant(name + " kF", kF);
    this.kS = new DoublePreferenceConstant(name + " kS", kS);
    this.iZone = new DoublePreferenceConstant(name + " iZone", iZone);
    this.iMax = new DoublePreferenceConstant(name + " iMax", iMax);
    this.tolerance = new DoublePreferenceConstant(name + " tolerance", tolerance);
  }

  /**
   * Constructor. Sets all defaults to 0.
   *
   * @param name The name to pre-pend to all preference names
   */
  public PIDPreferenceConstants(String name) {
    this(name, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  /** Updates all of the PID's preference constants */
  public void updateAll() {
    this.kP.update();
    this.kI.update();
    this.kD.update();
    this.kF.update();
    this.kS.update();
    this.iZone.update();
    this.iMax.update();
    this.tolerance.update();
  }

  /**
   * Get the kP preference constant.
   *
   * @return The kP preference constant
   */
  public DoublePreferenceConstant getKP() {
    return kP;
  }

  /**
   * Get the kI preference constant.
   *
   * @return The kI preference constant
   */
  public DoublePreferenceConstant getKI() {
    return kI;
  }

  /**
   * Get the kD preference constant.
   *
   * @return The kD preference constant
   */
  public DoublePreferenceConstant getKD() {
    return kD;
  }

  /**
   * Get the kF preference constant.
   *
   * @return The kF preference constant
   */
  public DoublePreferenceConstant getKF() {
    return kF;
  }

  /**
   * Get the kP preference constant.
   *
   * @return The kP preference constant
   */
  public DoublePreferenceConstant getKS() {
    return kS;
  }

  /**
   * Get the iZone preference constant.
   *
   * @return The iZone preference constant
   */
  public DoublePreferenceConstant getIZone() {
    return iZone;
  }

  /**
   * Get the iMax preference constant.
   *
   * @return The iMax preference constant
   */
  public DoublePreferenceConstant getIMax() {
    return iMax;
  }

  /**
   * Get the tolerance preference constant.
   *
   * @return The tolerance preference constant
   */
  public DoublePreferenceConstant getTolerance() {
    return tolerance;
  }

  /**
   * Adds the given handler to all of the underlying preference constants.
   *
   * @param handler The handler for all changes.
   */
  public void addChangeHandler(Consumer<Double> handler) {
    this.kP.addChangeHandler(handler);
    this.kI.addChangeHandler(handler);
    this.kD.addChangeHandler(handler);
    this.kF.addChangeHandler(handler);
    this.kS.addChangeHandler(handler);
    this.iZone.addChangeHandler(handler);
    this.iMax.addChangeHandler(handler);
    this.tolerance.addChangeHandler(handler);
  }
}
