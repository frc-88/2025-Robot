// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

/** Add your docs here. */
public class ReefTrax {
  private static final double REEF_X = 176.75;
  private static final double REEF_Y = 158.50;
  private static final double REEF_ROBOT_DIST = 50.12;
  private static final double POLE_SEPARATION = 12.94;
  private static final double ROBOT_SCORE_OFFSET = 6.877;

  private Pose2d[] baseReef = new Pose2d[12];

  private DoublePreferenceConstant[] blueOffsetForward = new DoublePreferenceConstant[12];
  private DoublePreferenceConstant[] blueOffsetRight = new DoublePreferenceConstant[12];
  private DoublePreferenceConstant[] redOffsetForward = new DoublePreferenceConstant[12];
  private DoublePreferenceConstant[] redOffsetRight = new DoublePreferenceConstant[12];

  public ReefTrax() {
    for (int pole = 1; pole < 13; pole++) {
      double targetAngle = Math.toRadians((Math.floor((pole - 1) / 2.0) + 1) * 60 % 360);
      double leftRightRobot = Math.pow(-1, pole) * POLE_SEPARATION / 2.0 - ROBOT_SCORE_OFFSET;
      // initialize base reef
      baseReef[pole - 1] =
          new Pose2d(
              Units.inchesToMeters(
                  REEF_X
                      + REEF_ROBOT_DIST * Math.cos(targetAngle)
                      - leftRightRobot * Math.sin(targetAngle)),
              Units.inchesToMeters(
                  REEF_Y
                      - REEF_ROBOT_DIST * Math.sin(targetAngle)
                      - leftRightRobot * Math.cos(targetAngle)),
              Rotation2d.fromRadians(targetAngle));

      // initialize blue/red forward/right offset preferences
      blueOffsetForward[pole - 1] =
          new DoublePreferenceConstant("ReefTrax/Blue/" + pole + "/Forward", 0.0);
      blueOffsetRight[pole - 1] =
          new DoublePreferenceConstant("ReefTrax/Blue/" + pole + "/Right", 0.0);
      redOffsetForward[pole - 1] =
          new DoublePreferenceConstant("ReefTrax/Red/" + pole + "/Forward", 0.0);
      redOffsetRight[pole - 1] =
          new DoublePreferenceConstant("ReefTrax/Red/" + pole + "/Right", 0.0);
    }
    // ReefTrax
    SmartDashboard.putData(
        "ReefTrax:Dump", new InstantCommand(() -> dumpReef()).ignoringDisable(true));
    SmartDashboard.putData(
        "ReefTrax:Reset Offsets",
        new InstantCommand(() -> resetReefOffsets()).ignoringDisable(true));
  }

  private boolean weAreRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public void resetReefOffsets() {
    for (int pole = 1; pole < 13; pole++) {
      blueOffsetForward[pole - 1].setValue(0.0);
      blueOffsetRight[pole - 1].setValue(0.0);
      redOffsetForward[pole - 1].setValue(0.0);
      redOffsetRight[pole - 1].setValue(0.0);
    }
  }

  public void dumpReef() {
    for (int pole = 1; pole < 13; pole++) {
      System.out.println("Pole " + pole + " = " + getPose(pole));
    }
  }

  public Pose2d getPose(int pole) {
    Translation2d offset;
    double targetAngle = Math.toRadians((Math.floor((pole - 1) / 2.0) + 1) * 60 % 360);
    System.out.println("Angle " + pole + " = " + targetAngle);

    if (weAreRed()) {
      offset =
          new Translation2d(
              -redOffsetForward[pole - 1].getValue() * Math.cos(targetAngle)
                  + redOffsetRight[pole - 1].getValue() * Math.sin(targetAngle),
              -redOffsetForward[pole - 1].getValue() * Math.sin(targetAngle)
                  - redOffsetRight[pole - 1].getValue() * Math.cos(targetAngle));

    } else {
      offset =
          new Translation2d(
              blueOffsetForward[pole - 1].getValue() * Math.cos(targetAngle)
                  + blueOffsetRight[pole - 1].getValue() * Math.sin(targetAngle),
              blueOffsetForward[pole - 1].getValue() * Math.sin(targetAngle)
                  - blueOffsetRight[pole - 1].getValue() * Math.cos(targetAngle));
    }

    return baseReef[pole - 1].transformBy(new Transform2d(offset, new Rotation2d()));
  }
}
