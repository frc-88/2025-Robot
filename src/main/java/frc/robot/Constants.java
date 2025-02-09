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
    public static final double ELEVATOR_L4_HEIGHT = 28.0;
  public static final double ELEVATOR_L3_HEIGHT = 13.25;
  public static final double ELEVATOR_L2_HEIGHT = 5.3;
  // ARMEVATOR
  public static final int ELEVATOR_MAIN_MOTOR = 3;
  public static final int ELEVATOR_FOLLOWER_MOTOR = 1;
  public static final int ELEVATOR_ARM_MOTOR = 7;
  public static final int ELEVATOR_MANIPULATOR_MOTOR = 5;
  public static final int ARM_LEFT_CANRANGE = 0;
  public static final int ARM_MIDDLE_CANRANGE = 10;
  public static final int ARM_RIGHT_CANRANGE = 3;

  public static final double ELEVATOR_ROTATIONS_TO_INCHES = ((2.256 * Math.PI) / 8.0);
  public static final double ARM_ROTATIONS_TO_DEGREES = (360.0 / 112.0);

  // DOGHOUSE
  public static final int DOGHOUSE_FUNNEL_MOTOR = 6;
}
