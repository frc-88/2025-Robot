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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 10.0;
  private static final double ANGLE_KD = 0.01;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double DRIVE_KP = 8.0;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_MAX_VELOCITY = 3.0;
  private static final double DRIVE_MAX_ACCELERATION = 3.5;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static double m_distance = 0.0;
  private static double m_x = 0.0;
  private static double m_y = 0.0;
  private static Rotation2d m_rotation = Rotation2d.kZero;
  private static Translation2d m_inital = Translation2d.kZero;
  private static Pose2d m_target = Pose2d.kZero;
  private static ProfiledPIDController m_sideController =
      new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 2.0));
  private static Debouncer m_targetDebouncer = new Debouncer(0.1);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command driveMoving(
      DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> angleSupplier, Drive drive) {

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(0.017);
    return Commands.run(
            () -> {
              // Convert to field relative speeds & send command
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(),
                      m_rotation.getRadians() + (drive.weAreRed() ? Math.PI : 0.0));
              ChassisSpeeds speeds = new ChassisSpeeds(m_x, m_y, omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
              m_x = x.getAsDouble();
              m_y = y.getAsDouble();
              m_rotation = angleSupplier.get();
            });
  }

  public static Command driveThenScore(Supplier<Pose2d> poseSupplier, Drive drive) {

    return new SequentialCommandGroup(
        drive(poseSupplier, drive)
            .until(
                () ->
                    m_targetDebouncer.calculate(
                            Math.abs(drive.getPoseFlipped().relativeTo(poseSupplier.get()).getX())
                                < 0.08)
                        && Math.abs(drive.getPoseFlipped().relativeTo(poseSupplier.get()).getY())
                            < 2.0

                /*&& (drive.getChassisTranslation().getAngle().getRadians()
                        - (poseSupplier.get().getRotation().getRadians()
                            - (Math.PI / 2.0))
                    < 0.14
                || drive.getChassisTranslation().getAngle().getRadians()
                        - (poseSupplier.get().getRotation().getRadians()
                            + (Math.PI / 2.0))
                Math.max(0.2, m_sideController.calculate(-drive.getPoseFlipped().getTranslation().getDistance(m_target.getTranslation())))
                    < 0.14)*/ ),
        driveMoving(
            () ->
                (drive.getPoseFlipped().relativeTo(m_target).getY() < 0.0 ? 1 : -1)
                    * 1.0
                    * poseSupplier.get().getRotation().plus(new Rotation2d(Math.PI / 2.0)).getCos(),
            () ->
                (drive.getPoseFlipped().relativeTo(m_target).getY() < 0.0 ? 1 : -1)
                    * 1.0
                    * poseSupplier.get().getRotation().plus(new Rotation2d(Math.PI / 2.0)).getSin(),
            () -> m_target.getRotation(),
            drive));
  }

  public static Command drive(Supplier<Pose2d> poseSupplier, Drive drive) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(0.017);

    ProfiledPIDController driveController =
        new ProfiledPIDController(
            0.9, 0.002, DRIVE_KD, new TrapezoidProfile.Constraints(1.25, 0.6));
    // driveController.setTolerance(0.02);

    return Commands.run(
            () -> {
              Pose2d targetPose = poseSupplier.get();
              Pose2d currentPose = drive.flipIfRed(drive.getPose());
              Translation2d target =
                  m_inital.interpolate(
                      m_target.getTranslation(),
                      Math.exp(
                          -3.0
                              * Math.abs(
                                  currentPose
                                      .relativeTo(m_target)
                                      .getX())) /*1.0 - (Math.abs(currentPose.relativeTo(m_target).getX()) / m_distance)*/);

              Translation2d direction = target.minus(currentPose.getTranslation());
              Rotation2d angle = direction.getAngle();

              double targetRotation =
                  (drive.getPoseFlipped().getTranslation().getDistance(m_target.getTranslation())
                          < 2.2
                      ? (m_target.getRotation().getRadians() + (drive.weAreRed() ? Math.PI : 0.0))
                      : Units.degreesToRadians(drive.aimAtReefCenter()));
              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), targetRotation);
              double speed =
                  driveController.calculate(-currentPose.getTranslation().getDistance(target), 0.0);

              ChassisSpeeds speeds =
                  new ChassisSpeeds(speed * angle.getCos(), speed * angle.getSin(), omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              m_target = poseSupplier.get();
              angleController.reset(drive.getRotation().getRadians());
              m_distance = Math.abs(drive.flipIfRed(drive.getPose()).relativeTo(m_target).getX());
              Pose2d targetPose = poseSupplier.get();
              Pose2d currentPose = drive.flipIfRed(drive.getPose());
              //   Translation2d velocityVector =
              //       new Translation2d(
              //               drive.getChassisVelocity().vxMetersPerSecond,
              //               drive.getChassisVelocity().vyMetersPerSecond)
              //           .rotateBy(currentPose.getRotation())
              //           .rotateBy(targetPose.getRotation().unaryMinus());

              // double angle = Math.atan2(velocityVector.getY(), velocityVector.getX());
              // double extra = currentPose.relativeTo(targetPose).getX() * Math.tan(angle);
              m_inital =
                  new Translation2d(0.0, currentPose.relativeTo(m_target).getY())
                      .rotateBy(m_target.getRotation())
                      .plus(m_target.getTranslation());
              ChassisSpeeds speeds = drive.getChassisVelocity();
              driveController.reset(
                  currentPose.getTranslation().getDistance(m_inital),
                  -Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
            });
  }

  public static Command driveToPose(Supplier<Pose2d> poseSupplier, Drive drive) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(0.017);

    ProfiledPIDController driveControllerX =
        new ProfiledPIDController(
            DRIVE_KP,
            0.003,
            DRIVE_KD,
            new TrapezoidProfile.Constraints(DRIVE_MAX_VELOCITY, DRIVE_MAX_ACCELERATION));
    driveControllerX.setTolerance(0.02);

    ProfiledPIDController driveControllerY =
        new ProfiledPIDController(
            DRIVE_KP,
            0.003,
            DRIVE_KD,
            new TrapezoidProfile.Constraints(DRIVE_MAX_VELOCITY, DRIVE_MAX_ACCELERATION));
    driveControllerY.setTolerance(0.02);

    // Construct command
    return Commands.run(
            () -> {
              Rotation2d rotation = poseSupplier.get().getRotation();
              Pose2d targetPose = poseSupplier.get();

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(),
                      rotation.getRadians() + (drive.weAreRed() ? Math.PI : 0.0));

              double velocityx =
                  driveControllerX.calculate(
                      drive.flipIfRed(drive.getPose()).getX(), targetPose.getX());
              double velocityy =
                  driveControllerY.calculate(
                      drive.flipIfRed(drive.getPose()).getY(), targetPose.getY());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds = new ChassisSpeeds(velocityx, velocityy, omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .until(
            () ->
                Math.abs(poseSupplier.get().getX() - drive.flipIfRed(drive.getPose()).getX()) < 0.01
                    && Math.abs(poseSupplier.get().getY() - drive.flipIfRed(drive.getPose()).getY())
                        < 0.01)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              ChassisSpeeds speeds = drive.getChassisVelocity();
              Translation2d translation =
                  new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                      .rotateBy(drive.flipIfRed(drive.getPose()).getRotation());
              angleController.reset(drive.getRotation().getRadians());
              driveControllerX.reset(drive.flipIfRed(drive.getPose()).getX(), translation.getX());
              driveControllerY.reset(drive.flipIfRed(drive.getPose()).getY(), translation.getY());
            });
  }
  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(1.0);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
