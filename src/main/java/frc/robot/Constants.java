// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final String RIO_CANBUS = "rio";

  public static final double ARM_L4_ANGLE = 24.5;
  public static final double ELEVATOR_L4_HEIGHT = 28.;
  public static final double ELEVATOR_L3_HEIGHT = 13.25;
  public static final double ELEVATOR_L2_HEIGHT = 5.3;
  // ARMEVATOR
  public static final int ELEVATOR_MAIN_MOTOR = 3;
  public static final int ELEVATOR_FOLLOWER_MOTOR = 1;
  public static final int ELEVATOR_ARM_MOTOR = 7;
  public static final int ELEVATOR_MANIPULATOR_MOTOR = 5;
  public static final int DOGHOUSE_CANRANGE = 10;
  public static final int CORAL_CANRANGE = 3;
  public static final int ARM_RIGHT_CANRANGE = 3;

  public static final double ELEVATOR_ROTATIONS_TO_INCHES = ((2.256 * Math.PI) / 8.0);
  public static final double ARM_ROTATIONS_TO_DEGREES = (360.0 / 112.0);

  // DOGHOUSE
  public static final int DOGHOUSE_FUNNEL_MOTOR = 6;

  // CLIMBER
  public static final int CLIMBER_GRIPPER_MOTOR = 17;
  public static final int CLIMBER_GAS_MOTOR = 12;
  public static final int CLIMBER_ENCODER = 3;
  public static final int CLIMBER_CANRANGE = 5;
  public static final double PIVOT_MOTOR_ROTATIONS_TO_CLIMBER_POSITION = (360.0 / 196.0);
  public static final double GRIPPER_MOTOR_ROTATIONS_TO_ANGLE = (360 / 49.0);
  public static final double GAS_MOTOR_ROTATIONS_TO_LENGTH = (1.39 / 28.0);
  public static final double CLIMBER_ENCODER_ROTATIONS_TO_ANGLE = 360;

  // LIGHTS
  public static final int CANdleID = 0;

  // CONTROLLER
  public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
  public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
  public static final int BlockButton = XboxController.Button.kStart.value;
  public static final int MaxBrightnessAngle = 90;
  public static final int MidBrightnessAngle = 180;
  public static final int ZeroBrightnessAngle = 270;
  public static final int VbatButton = XboxController.Button.kA.value;
  public static final int V5Button = XboxController.Button.kB.value;
  public static final int CurrentButton = XboxController.Button.kX.value;
  public static final int TemperatureButton = XboxController.Button.kY.value;
}
