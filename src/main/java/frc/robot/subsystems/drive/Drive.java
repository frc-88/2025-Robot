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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// we stand on shoulders
// of mechanical giants
// what an advantage

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally

  private ArrayList<PathPlannerPath> m_paths = new ArrayList<>();

  public int m_currentPathEven = 10;
  public int m_currentPathOdd = 10;
  public Pose2d nextPose = new Pose2d();
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private boolean m_autoaim = true;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    for (int i = 1; i < 13; i++) {
      try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Score " + i);
        m_paths.add(path);
        System.out.println("Loaded path " + i);
      } catch (Exception e) {
        Command autoPath = new WaitCommand(1.0);
        // m_paths.add(autoPath);
        System.err.println("Exception loading auto path");
        e.printStackTrace();
      }
    }
  }

  public boolean isReady() {
    return gyroInputs.connected
        && modules[0].isReady()
        && modules[1].isReady()
        && modules[2].isReady()
        && modules[3].isReady();
  }

  private boolean weAreRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  private Pose2d flipIfRed(Pose2d pose) {
    if (weAreRed()) {
      return pose.relativeTo(
          new Pose2d(
              Constants.FIELD_LENGTH,
              Constants.FIELD_WIDTH,
              new Rotation2d(Units.degreesToRadians(180))));
    } else {
      return pose;
    }
  }

  private double aimAtStation() {
    if (getPose().getY() < (Constants.FIELD_WIDTH / 2.0)) {
      return weAreRed() ? 135.0 : 45;
    } else {
      return weAreRed() ? -135.0 : -45.0;
    }
  }

  private double getAngleToReef(Pose2d pose) {
    Translation2d position = flipIfRed(pose).relativeTo(Constants.REEF_POSE).getTranslation();
    double x = position.getX();
    double y = position.getY();
    return Units.radiansToDegrees(Math.atan2(y, x));
  }

  private boolean isNearReef() {
    return flipIfRed(getPose()).getTranslation().getDistance(Constants.REEF_POSE.getTranslation())
        < 2.0;
  }

  public boolean isShootingDistance() {
    return flipIfRed(getPose()).getTranslation().getDistance(Constants.REEF_POSE.getTranslation())
        < 1.40;
  }

  private Command getPath(int i) {
    try {
      return AutoBuilder.followPath(m_paths.get(i - 1));
    } catch (IndexOutOfBoundsException e) {
      return new WaitCommand(1.0);
    }
  }

  private Command drivePathFind(int i) {
    try {
      boolean present = m_paths.get(i - 1).getStartingHolonomicPose().isPresent();
      Pose2d pose = present ? m_paths.get(i - 1).getStartingHolonomicPose().get() : getPose();
      return AutoBuilder.pathfindToPose(flipIfRed(pose), Constants.CONSTRAINTS);
    } catch (IndexOutOfBoundsException e) {
      return new WaitCommand(1.0);
    }
  }

  private double aimAtReef() {
    double angle = getAngleToReef(nextPose());
    if (angle < 30.0 && angle > -30.0) {
      angle = weAreRed() ? 0.0 : 180.0;
    } else if (angle > 30.0 && angle < 90.0) {
      angle = weAreRed() ? 60.0 : -120.0;
    } else if (angle > 90.0 && angle < 150.0) {
      angle = weAreRed() ? 120.0 : -60.0;
    } else if (angle > 150.0 || angle < -150.0) {
      angle = weAreRed() ? 180.0 : 0.0;
    } else if (angle > -150.0 && angle < -90.0) {
      angle = weAreRed() ? -120.0 : 60.0;
    } else if (angle > -90.0 && angle < -30.0) {
      angle = weAreRed() ? -60.0 : 120.0;
    }
    return angle;
  }

  public void enableAutoAim() {
    m_autoaim = true;
  }

  public void disableAutoAim() {
    m_autoaim = false;
  }

  public double aimAtExpectedTarget(BooleanSupplier hasCoral) {
    if (!m_autoaim) {
      return getPose().getRotation().getDegrees();
    } else if (hasCoral.getAsBoolean()) {
      return aimAtReef();
    } else if (!isNearReef()) {
      return aimAtStation();
    } else {
      return getPose().getRotation().getDegrees();
    }
  }

  public Pose2d nextPose() {
    Pose2d current = getPose();
    double x = getChassisSpeeds().vxMetersPerSecond;
    double y = getChassisSpeeds().vyMetersPerSecond;

    return new Pose2d(
        current.getX() + (x * 0.2), current.getY() + (y * 0.2), current.getRotation());
  }

  private int getTargetSector() {
    int target = 0;
    double angle = getAngleToReef(nextPose());
    if (angle < 30.0 && angle > -30.0) {
      target = 6;
    } else if (angle > 30.0 && angle < 90.0) {
      target = 5;
    } else if (angle > 90.0 && angle < 150.0) {
      target = 4;
    } else if (angle > 150.0 || angle < -150.0) {
      target = 3;
    } else if (angle > -150.0 && angle < -90.0) {
      target = 2;
    } else if (angle > -90.0 && angle < -30.0) {
      target = 1;
    }
    return target;
  }

  public Command scoreOnReef(boolean odd) {
    return new ConditionalCommand(
        odd ? getPath(1) : getPath(2),
        new ConditionalCommand(
            odd ? getPath(3) : getPath(4),
            new ConditionalCommand(
                odd ? getPath(5) : getPath(6),
                new ConditionalCommand(
                    odd ? getPath(7) : getPath(8),
                    new ConditionalCommand(
                        odd ? getPath(9) : getPath(10),
                        new ConditionalCommand(
                            odd ? getPath(11) : getPath(12),
                            new WaitCommand(0.5),
                            () -> getTargetSector() == 6),
                        () -> getTargetSector() == 5),
                    () -> getTargetSector() == 4),
                () -> getTargetSector() == 3),
            () -> getTargetSector() == 2),
        () -> getTargetSector() == 1);
  }

  public Command pathFind(boolean odd) {
    return new ConditionalCommand(
        odd ? drivePathFind(1) : drivePathFind(2),
        new ConditionalCommand(
            odd ? drivePathFind(3) : drivePathFind(4),
            new ConditionalCommand(
                odd ? drivePathFind(5) : drivePathFind(6),
                new ConditionalCommand(
                    odd ? drivePathFind(7) : drivePathFind(8),
                    new ConditionalCommand(
                        odd ? drivePathFind(9) : drivePathFind(10),
                        new ConditionalCommand(
                            odd ? drivePathFind(11) : drivePathFind(12),
                            new WaitCommand(0.5),
                            () -> getTargetSector() == 6),
                        () -> getTargetSector() == 5),
                    () -> getTargetSector() == 4),
                () -> getTargetSector() == 3),
            () -> getTargetSector() == 2),
        () -> getTargetSector() == 1);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    double angle = getAngleToReef(getPose());
    if (angle < 30.0 && angle > -30.0) {
      m_currentPathOdd = 11;
    } else if (angle > 30.0 && angle < 90.0) {
      m_currentPathOdd = 9;
    } else if (angle > 90.0 && angle < 150.0) {
      m_currentPathOdd = 7;
    } else if (angle > 150.0 || angle < -150.0) {
      m_currentPathOdd = 5;
    } else if (angle > -150.0 && angle < -90.0) {
      m_currentPathOdd = 3;
    } else if (angle > -90.0 && angle < -30.0) {
      m_currentPathOdd = 1;
    } else {
      m_currentPathOdd = 2;
    }
    SmartDashboard.putNumber("CurrentPathOdd", m_currentPathOdd);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
