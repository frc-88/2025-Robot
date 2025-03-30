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
import java.util.Map;

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
  public static final double ARM_L4_SAFE_ANGLE = 23.0;
  public static final double ELEVATOR_L4_HEIGHT = 28.4;
  public static final double ELEVATOR_L3_HEIGHT = 14.0 - 0.125;
  public static final double ELEVATOR_L2_HEIGHT = 5.375 + 0.25;
  public static final double ALGAE_STOW_ANGLE = 0.0;
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
  public static final double ARM_ROTATIONS_TO_DEGREES = (360.0 / 48.0);

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
  public static final PathConstraints CONSTRAINTS = new PathConstraints(2.75, 2.25, 8.0, 20.0);

  public static final double FIELD_WIDTH = 8.05;
  public static final double FIELD_LENGTH = 17.55;

  public static final Pose2d REEF_POSE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(176.75), Units.feetToMeters(13.5)),
          Rotation2d.fromDegrees(0.0));

  // Theoretical positions
  // Pole PoseX(m) PoseY(m)
  // 1 5.441 3.105
  // 2 5.155 2.940
  // 3 4.163 2.765
  // 4 3.877 2.930
  // 5 3.229 3.701
  // 6 3.229 4.031
  // 7 3.573 4.978
  // 8 3.859 5.143
  // 9 4.851 5.319
  // 10 5.137 5.154
  // 11 5.785 4.382
  // 12 5.785 4.052

  // WPI
  // 5.427 3.028
  // 5.142 2.863
  // 4.133 2.713
  // 3.847 2.878
  // 3.169 3.701
  // 3.169 4.031
  // 3.543 5.031
  // 3.829 5.196
  // 4.881 5.371
  // 5.167 5.206
  // 5.845 4.382
  // 5.845 4.052

  public static final Map<Integer, Pose2d> REEF_CORAL_POSES_BLUE =
      Map.ofEntries(
          Map.entry(1, new Pose2d(5.427, 3.028, new Rotation2d(Units.degreesToRadians(120.0)))),
          Map.entry(2, new Pose2d(5.168, 2.878, new Rotation2d(Units.degreesToRadians(120.0)))),
          Map.entry(3, new Pose2d(4.133, 2.713, new Rotation2d(Units.degreesToRadians(60.0)))),
          Map.entry(4, new Pose2d(3.837, 2.860, new Rotation2d(Units.degreesToRadians(60.0)))),
          Map.entry(5, new Pose2d(3.149, 3.701, new Rotation2d())),
          Map.entry(6, new Pose2d(3.149, 4.031, new Rotation2d())),
          Map.entry(7, new Pose2d(3.485, 5.032, new Rotation2d(Units.degreesToRadians(-60.0)))),
          Map.entry(8, new Pose2d(3.785, 5.170, new Rotation2d(Units.degreesToRadians(-60.0)))),
          Map.entry(9, new Pose2d(4.881, 5.371, new Rotation2d(Units.degreesToRadians(-120.0)))),
          Map.entry(10, new Pose2d(5.167, 5.206, new Rotation2d(Units.degreesToRadians(-120.0)))),
          Map.entry(11, new Pose2d(5.845, 4.382, new Rotation2d(Units.degreesToRadians(180.0)))),
          Map.entry(12, new Pose2d(5.845, 4.052, new Rotation2d(Units.degreesToRadians(180.0)))));

  public static final Map<Integer, Pose2d> REEF_CORAL_POSES_RED =
      Map.ofEntries(
          Map.entry(1, new Pose2d(5.427, 3.028, new Rotation2d(Units.degreesToRadians(120.0)))),
          Map.entry(2, new Pose2d(5.168, 2.878, new Rotation2d(Units.degreesToRadians(120.0)))),
          Map.entry(3, new Pose2d(4.133, 2.713, new Rotation2d(Units.degreesToRadians(60.0)))),
          Map.entry(4, new Pose2d(3.837, 2.860, new Rotation2d(Units.degreesToRadians(60.0)))),
          Map.entry(5, new Pose2d(3.149, 3.701, new Rotation2d())),
          Map.entry(6, new Pose2d(3.149, 4.031, new Rotation2d())),
          Map.entry(7, new Pose2d(3.485, 5.032, new Rotation2d(Units.degreesToRadians(-60.0)))),
          Map.entry(8, new Pose2d(3.785, 5.170, new Rotation2d(Units.degreesToRadians(-60.0)))),
          Map.entry(9, new Pose2d(4.881, 5.371, new Rotation2d(Units.degreesToRadians(-120.0)))),
          Map.entry(10, new Pose2d(5.167, 5.206, new Rotation2d(Units.degreesToRadians(-120.0)))),
          Map.entry(11, new Pose2d(5.845, 4.382, new Rotation2d(Units.degreesToRadians(180.0)))),
          Map.entry(12, new Pose2d(5.845, 4.052, new Rotation2d(Units.degreesToRadians(180.0)))));

  public static Pose2d SECTOR3ALGAE = new Pose2d(3.2, 3.85, new Rotation2d());
  public static Pose2d SECTOR2ALGAE =
      new Pose2d(3.892, 2.778, new Rotation2d(Units.degreesToRadians(60.0)));
  public static Pose2d SECTOR6ALGAE =
      new Pose2d(5.7825, 4.16, new Rotation2d(Units.degreesToRadians(180.0)));
  public static Pose2d SECTOR1ALGAE =
      new Pose2d(5.298, 3.0225, new Rotation2d(Units.degreesToRadians(120.0)));
  public static Pose2d SECTOR5ALGAE =
      new Pose2d(5.002, 5.255, new Rotation2d(Units.degreesToRadians(-120.0)));
  public static Pose2d SECTOR4ALGAE =
      new Pose2d(3.716, 5.0605, new Rotation2d(Units.degreesToRadians(-60.0)));

  public static Pose2d SECTOR3L1 = new Pose2d(3.2, 3.9, new Rotation2d());
  public static Pose2d SECTOR2L1 =
      new Pose2d(3.892, 2.778, new Rotation2d(Units.degreesToRadians(60.0)));
  public static Pose2d SECTOR6L1 =
      new Pose2d(5.7825, 4.16, new Rotation2d(Units.degreesToRadians(180.0)));
  public static Pose2d SECTOR1L1 =
      new Pose2d(5.298, 3.0225, new Rotation2d(Units.degreesToRadians(120.0)));
  public static Pose2d SECTOR5L1 =
      new Pose2d(5.002, 5.255, new Rotation2d(Units.degreesToRadians(-120.0)));
  public static Pose2d SECTOR4L1 =
      new Pose2d(3.716, 5.0605, new Rotation2d(Units.degreesToRadians(-60.0)));

  public static final Pose2d POSE5MOVING = new Pose2d(3.08, 3.154, new Rotation2d());
  public static final Pose2d POSE6MOVING = new Pose2d(3.08, 4.896, new Rotation2d());
}
