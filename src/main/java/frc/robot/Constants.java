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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

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

  public static final double ARM_L4_ANGLE = 37.0;
  public static final double ELEVATOR_L4_HEIGHT = 28.875;
  public static final double ELEVATOR_L3_HEIGHT = 14.0 - 0.125;
  public static final double ELEVATOR_L2_HEIGHT = 5.375 + 0.25;
  public static final double ALGAE_STOW_ANGLE = 35.0;
  // ARMEVATOR
  public static final int ELEVATOR_MAIN_MOTOR = 2;
  public static final int ELEVATOR_FOLLOWER_MOTOR = 3;
  public static final int ELEVATOR_ARM_MOTOR = 15;
  public static final int ELEVATOR_MANIPULATOR_MOTOR = 16;
  public static final int ELEVATOR_ENCODER = 15;
  public static final int DOGHOUSE_CANRANGE = 10;
  public static final int CORAL_CANRANGE = 3;
  public static final int REEF_CANRANGE = 5;

  public static final double ELEVATOR_ROTATIONS_TO_INCHES = ((2.256 * Math.PI) / 8.0);
  public static final double ARM_ROTATIONS_TO_DEGREES = (360.0 / 112.0);

  // DOGHOUSE
  public static final int DOGHOUSE_FUNNEL_MOTOR = 6;

  // CLIMBER
  public static final int CLIMBER_GRIPPER_MOTOR = 17;
  public static final int CLIMBER_GAS_MOTOR = 5;
  public static final int CLIMBER_ENCODER = 12;
  public static final int CLIMBER_CANRANGE = 17;
  public static final double PIVOT_MOTOR_ROTATIONS_TO_CLIMBER_POSITION = (360.0 / 196.0);
  public static final double GRIPPER_MOTOR_ROTATIONS_TO_ANGLE = (360 / 21.0);
  public static final double GAS_MOTOR_ROTATIONS_TO_LENGTH = (8.0 / 180.0);
  public static final double CLIMBER_ENCODER_ROTATIONS_TO_ANGLE = 360;

  // LIGHTS
  public static final int CANDLE_ID = 0;

  // AUTOMATION
  public static final PathConstraints CONSTRAINTS = new PathConstraints(3.0, 3.0, 8.0, 20.0);

  public static final double FIELD_WIDTH = 8.05;
  public static final double FIELD_LENGTH = 17.55;

  public static final Pose2d REEF_POSE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(176.75), Units.feetToMeters(13.5)),
          Rotation2d.fromDegrees(0.0));
}
