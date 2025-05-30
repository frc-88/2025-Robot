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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.ReefTrax;
import java.util.ArrayList;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
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
  public int m_currentPose = 0;
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

  Field2d m_odomPose = new Field2d();

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

  private final Map<Integer, Pose2d> REEF_CORAL_POSES;

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
  private ReefTrax reeftrax = new ReefTrax();
  private int jumpCounter = 0;

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

    REEF_CORAL_POSES =
        weAreRed() ? Constants.REEF_CORAL_POSES_RED : Constants.REEF_CORAL_POSES_BLUE;

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
            new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
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

  public boolean weAreRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  private Pose2d flipPose(Pose2d pose) {
    return pose.relativeTo(
        new Pose2d(
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            new Rotation2d(Units.degreesToRadians(180))));
  }

  public Pose2d flipIfRed(Pose2d pose) {
    return weAreRed() ? flipPose(pose) : pose;
  }

  public Pose2d getPoseFlipped() {
    return flipIfRed(getPose());
  }

  public Translation2d getChassisTranslation() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
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

  public boolean isElevatorDistance(double distance) {
    return flipIfRed(getPose()).getTranslation().getDistance(Constants.REEF_POSE.getTranslation())
        < distance;
  }

  public boolean isElevatorDistance() {
    return isElevatorDistance(2.5);
  }

  public boolean isAtTarget(boolean odd) {
    return flipIfRed(getPose())
                .getTranslation()
                .getDistance(getTargetPoseFromSector(odd).getTranslation())
            < 0.07
        && Math.abs(flipIfRed(getPose()).relativeTo(getTargetPose()).getY()) < 0.1;
  }

  public boolean isAtTarget6() {
    return Math.abs(flipIfRed(getPose()).relativeTo(REEF_CORAL_POSES.get(6)).getY()) < 0.12;
  }

  public boolean isAtTargetPose(Pose2d pose) {
    return Math.abs(flipIfRed(getPose()).relativeTo(pose).getY()) < 0.10;
  }

  public boolean isFacingForward() {
    return Math.abs(flipIfRed(getPose()).getRotation().getDegrees()) < 20.0;
  }

  public boolean shouldShootAlgae() {
    return flipIfRed(getPose()).getX() > 6.55
        && flipIfRed(getPose()).getY() > (Constants.FIELD_WIDTH / 2.0);
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
      return AutoBuilder.pathfindToPoseFlipped(pose, Constants.CONSTRAINTS);
    } catch (IndexOutOfBoundsException e) {
      return new WaitCommand(1.0);
    }
  }

  public Command pathFind(int i) {
    return new InstantCommand(() -> m_currentPose = i)
        .andThen(
            new ConditionalCommand(
                AutoBuilder.pathfindToPoseFlipped(reeftrax.getRedPose(i), Constants.CONSTRAINTS),
                AutoBuilder.pathfindToPoseFlipped(reeftrax.getBluePose(i), Constants.CONSTRAINTS),
                () -> weAreRed()));
  }

  public Command pathFindAuto(int i) {
    return new InstantCommand(() -> m_currentPose = i)
        .andThen(
            AutoBuilder.pathfindToPoseFlipped(REEF_CORAL_POSES.get(i), Constants.AUTO_CONSTRAINTS));
  }

  public Command pathFindToL1(int i) {
    return new InstantCommand(() -> m_currentPose = i)
        .andThen(
            AutoBuilder.pathfindToPoseFlipped(
                Constants.REEF_CORAL_L1_POSES.get(i), Constants.CONSTRAINTS));
  }

  public Command pathFindAlgae(int sector) {
    if (sector == 3) {
      return AutoBuilder.pathfindToPoseFlipped(Constants.SECTOR3ALGAE, Constants.CONSTRAINTS);
    } else if (sector == 2) {
      return AutoBuilder.pathfindToPoseFlipped(Constants.SECTOR2ALGAE, Constants.CONSTRAINTS);
    } else if (sector == 1) {
      return AutoBuilder.pathfindToPoseFlipped(Constants.SECTOR1ALGAE, Constants.CONSTRAINTS);
    } else if (sector == 4) {
      return AutoBuilder.pathfindToPoseFlipped(Constants.SECTOR4ALGAE, Constants.CONSTRAINTS);
    } else if (sector == 5) {
      return AutoBuilder.pathfindToPoseFlipped(Constants.SECTOR5ALGAE, Constants.CONSTRAINTS);
    } else if (sector == 6) {
      return AutoBuilder.pathfindToPoseFlipped(Constants.SECTOR6ALGAE, Constants.CONSTRAINTS);
    } else {
      return new WaitCommand(1.0);
    }
  }

  public Pose2d getTargetPose() {
    if (m_currentPose >= 1 & m_currentPose <= 12) {
      return reeftrax.getPose(m_currentPose);
    } else {
      return new Pose2d();
    }
  }

  public Pose2d getL1Pose() {
    return Constants.REEF_CORAL_L1_POSES.get(getTargetSector());
  }

  private Command pathFindMoving(int i) {
    if (i == 5) {
      return AutoBuilder.pathfindToPoseFlipped(REEF_CORAL_POSES.get(5), Constants.CONSTRAINTS, 0.3);
    } else {
      return AutoBuilder.pathfindToPoseFlipped(REEF_CORAL_POSES.get(6), Constants.CONSTRAINTS, 0.3);
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

  public double aimAtReefCenter() {
    double angle = getAngleToReef(getPose());
    angle = weAreRed() ? angle : angle + 180.0;
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
    } else if (flipIfRed(getPose()).getX() > 7.0) {
      return getPose().getRotation().getDegrees();
    } else if (getPoseFlipped()
            .getTranslation()
            .getDistance(Constants.PROCESSOR_POSITION.getTranslation())
        < 1.5) {
      return getPose().getRotation().getDegrees();
    } else if (!isNearReef() && !(flipIfRed(getPose()).getX() > 2.5) && !hasCoral.getAsBoolean()) {
      return aimAtStation();
    } else {
      // return aimAtReef();
      return aimAtReefCenter();
    }
  }

  public Pose2d nextPose(double lookAhead) {
    Pose2d current = getPose();
    double x = getChassisSpeeds().vxMetersPerSecond;
    double y = getChassisSpeeds().vyMetersPerSecond;

    return new Pose2d(
        current.getX() + (x * lookAhead), current.getY() + (y * lookAhead), current.getRotation());
  }

  public Pose2d nextPose() {
    return nextPose(0.1);
  }

  public int getTargetSector() {
    double angle = getAngleToReef(nextPose());
    if (angle < 30.0 && angle > -30.0) {
      return 6;
    } else if (angle > 30.0 && angle < 90.0) {
      return 5;
    } else if (angle > 90.0 && angle < 150.0) {
      return 4;
    } else if (angle > 150.0 || angle < -150.0) {
      return 3;
    } else if (angle > -150.0 && angle < -90.0) {
      return 2;
    } else if (angle > -90.0 && angle < -30.0) {
      return 1;
    }
    return 0;
  }

  public Pose2d getReefTraxPose(int pole) {
    return reeftrax.getRedPose(pole);
  }

  public int getTargetPositionFromSector(boolean odd) {
    return getTargetSector() * 2 - (odd ? 1 : 0);
  }

  public Pose2d getTargetPoseFromSector(boolean odd) {
    return REEF_CORAL_POSES.get(getTargetPositionFromSector(odd));
  }

  public double getDistanceToPose(boolean odd, IntSupplier mode) {
    if (mode.getAsInt() == 2 || mode.getAsInt() == 3 || mode.getAsInt() == 4) {
      return flipIfRed(getPose())
          .relativeTo(getTargetPoseFromSector(odd))
          .getTranslation()
          .getNorm();
    } else {
      return flipIfRed(getPose()).relativeTo(getL1Pose()).getTranslation().getNorm();
    }
  }

  public Pose2d getTargetAlgaePoseFromSector() {
    double angle = getAngleToReef(nextPose());
    if (angle < 30.0 && angle > -30.0) {
      return Constants.SECTOR6ALGAE;
    } else if (angle > 30.0 && angle < 90.0) {
      return Constants.SECTOR5ALGAE;
    } else if (angle > 90.0 && angle < 150.0) {
      return Constants.SECTOR4ALGAE;
    } else if (angle > 150.0 || angle < -150.0) {
      return Constants.SECTOR3ALGAE;
    } else if (angle > -150.0 && angle < -90.0) {
      return Constants.SECTOR2ALGAE;
    } else if (angle > -90.0 && angle < -30.0) {
      return Constants.SECTOR1ALGAE;
    }
    return Pose2d.kZero;
  }

  public int getTargetSectorNow() {
    double angle = getAngleToReef(getPose());
    if (angle < 30.0 && angle > -30.0) {
      return 6;
    } else if (angle > 30.0 && angle < 90.0) {
      return 5;
    } else if (angle > 90.0 && angle < 150.0) {
      return 4;
    } else if (angle > 150.0 || angle < -150.0) {
      return 3;
    } else if (angle > -150.0 && angle < -90.0) {
      return 2;
    } else if (angle > -90.0 && angle < -30.0) {
      return 1;
    }
    return 0;
  }

  public int getTargetPositionFromSectorNow(boolean odd) {
    return getTargetSectorNow() * 2 - (odd ? 1 : 0);
  }

  public Command scoreOnReef(boolean odd) {
    return new SelectCommand<>(
        IntStream.rangeClosed(1, 12).boxed().collect(Collectors.toMap(i -> i, this::getPath)),
        () -> getTargetPositionFromSector(odd));
  }

  public Command pathFind(boolean odd) {
    return new SelectCommand<>(
        IntStream.rangeClosed(1, 12).boxed().collect(Collectors.toMap(i -> i, this::drivePathFind)),
        () -> getTargetPositionFromSector(odd));
  }

  public Command reef(boolean odd, IntSupplier mode) {
    return new ConditionalCommand(
        new SelectCommand<>(
            IntStream.rangeClosed(1, 6)
                .boxed()
                .collect(Collectors.toMap(i -> i, this::pathFindToL1)),
            () -> getTargetSector()),
        new SelectCommand<>(
            IntStream.rangeClosed(1, 12).boxed().collect(Collectors.toMap(i -> i, this::pathFind)),
            () -> getTargetPositionFromSectorNow(odd)),
        () -> mode.getAsInt() == 1);
  }

  public Command algae() {
    return new SelectCommand<>(
        IntStream.rangeClosed(1, 6).boxed().collect(Collectors.toMap(i -> i, this::pathFindAlgae)),
        this::getTargetSectorNow);
  }

  public Command reefMoving(boolean odd) {
    return odd ? pathFind(5) : pathFind(6);
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
    SmartDashboard.putNumber(
        "Error x", Math.abs(REEF_CORAL_POSES.get(5).getX() - flipIfRed(getPose()).getX()));
    SmartDashboard.putNumber(
        "Error y", Math.abs(REEF_CORAL_POSES.get(5).getY() - flipIfRed(getPose()).getY()));
    m_odomPose.setRobotPose(getPose());
    SmartDashboard.putData("Odometry Pose", m_odomPose);
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

  public ChassisSpeeds getChassisVelocity() {
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

    if (getPose().getTranslation().getDistance(visionRobotPoseMeters.getTranslation()) > 1.0) {
      // big jump
      Logger.recordOutput("Drive/Jump Count", ++jumpCounter);
      Logger.recordOutput("Drive/Jump Pose", getPose());
    }

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
