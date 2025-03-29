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

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Doghouse;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Lights lights;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private CommandGenericHID buttons = new CommandGenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public Doghouse m_doghouse = new Doghouse();
  public Armevator m_armevator = new Armevator(() -> !m_doghouse.isBlocked());
  public Climber climber = new Climber();

  public LocalADStarAK pathFinder = new LocalADStarAK();

  public Trigger driveOnCoral = new Trigger(() -> hasCoralDebounced());
  public Trigger shouldShootAlgae;
  public Trigger shouldStow;
  Timer timer = new Timer();
  private DoublePreferenceConstant p_amplitude = new DoublePreferenceConstant("Aplitude", 0);
  private DoublePreferenceConstant p_frequency = new DoublePreferenceConstant("Wavelength", 0);

  private Debouncer reefDebouncer = new Debouncer(0.2);
  private boolean shootingAlgae = false;
  // public Trigger atL4 = new Trigger(() -> hasCoralDebounced() && m_armevator.atL4());
  // public Trigger atL3 = new Trigger(() -> hasCoralDebounced() && m_armevator.atL3());
  // public Trigger atL2 =
  //     new Trigger(
  //         () -> hasCoralDebounced() && m_armevator.atL2() && m_doghouse.getIsReefDetected());
  public int mode = 3;

  public RobotContainer() {
    timer.start();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    shouldShootAlgae = new Trigger(() -> m_doghouse.isAlgaeMode() && drive.shouldShootAlgae());
    shouldStow = new Trigger(() -> drive.shouldShootAlgae() && shootingAlgae);

    registerNamedCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // ready functions replaced with lambdas returning true for now...need to be implemented in each
    // subsystem
    lights =
        new Lights(
            drive::isReady,
            m_armevator::isReady,
            m_doghouse::isReady,
            climber::isReady,
            vision::isReady,
            m_doghouse::hasCoral,
            m_armevator::isElevatorDown,
            () -> autoChooser.get().getName());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureDashboardButtons();
    configureButtonBox();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot", shootCommand(0.5));
    NamedCommands.registerCommand("NetFling", netflingCommand());
    NamedCommands.registerCommand(
        "Get Coral", m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown()));
    NamedCommands.registerCommand(
        "Arm Go To Zero", m_armevator.armGoToZeroFactory().withTimeout(0.5));
    NamedCommands.registerCommand("L4", m_armevator.L4Factory().withTimeout(1.25));
    NamedCommands.registerCommand("L4 Mode", new InstantCommand(() -> mode = 4));
    NamedCommands.registerCommand("L3 Mode", new InstantCommand(() -> mode = 3));
    NamedCommands.registerCommand("L2 Mode", new InstantCommand(() -> mode = 2));
    NamedCommands.registerCommand("Stow", m_armevator.stowFactory().withTimeout(1.0));
    NamedCommands.registerCommand("Armevator Calibration", m_armevator.calibrateBothFactory());
    NamedCommands.registerCommand("Score Odd", scoreNoShoot(true));
    NamedCommands.registerCommand("Score Even", scoreNoShoot(false));
    NamedCommands.registerCommand("Reef Even", reef(false, 0.5));
    NamedCommands.registerCommand("Reef Odd", reef(true, 0.5));

    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();
  }

  public void configureDashboardButtons() {

    climber.shouldNeutral().onTrue(climber.gasMotorNeutralModeFactory().ignoringDisable(true));
    // .onFalse(climber.gasMotorBrakeModeFactory().ignoringDisable(true));
    climber
        .shouldGripperClose()
        .onTrue(
            climber
                .closeGrabberFactory()
                .alongWith(
                    new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 1))
                        .andThen(new WaitCommand(2.0))
                        .andThen(
                            new InstantCommand(
                                () -> controller.setRumble(RumbleType.kBothRumble, 0))))
                .alongWith(m_doghouse.stopAllFactory()));

    climber.shouldSoftCloseTrigger.onTrue(
        climber.softCloseFactory().alongWith(m_doghouse.stopAllFactory()));
    m_armevator.m_shouldCalibrate.onTrue(m_armevator.elevatorCalibrateFactory());
    shouldShootAlgae.onTrue(
        new InstantCommand(() -> shootingAlgae = true)
            .andThen(
                m_doghouse
                    .shootAlgaeFactory()
                    .andThen(m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown()))
                    .alongWith(
                        DriveCommands.joystickDriveAtAngle(
                            drive,
                            () -> -controller.getLeftY(),
                            () -> -controller.getLeftX(),
                            () ->
                                Rotation2d.fromDegrees(
                                    drive.aimAtExpectedTarget(() -> m_doghouse.hasCoral()))),
                        m_armevator.shootInNetFactory())));
    shouldStow.onFalse(
        new InstantCommand(() -> shootingAlgae = false).andThen(m_armevator.stowFactory()));
    // atL2.onTrue(m_doghouse.shootFactory());
    // .onFalse(climber.setNotGrabbed());
    // climber.forceCloseOnDisable().onTrue(climber.climbOnDisable().ignoringDisable(true));

    SmartDashboard.putData("Calibrate Elevator", m_armevator.calibrateElevatorFactory());
    SmartDashboard.putData("Calibrate Arm", m_armevator.calibrateArmFactory());
    SmartDashboard.putData("Set Position Elevator", m_armevator.setElevatorPostionFactory());
    SmartDashboard.putData("Stop Elevator", m_armevator.stopElevatorFactory());
    SmartDashboard.putData("Set Position Arm", m_armevator.setArmPostionFactory());
    SmartDashboard.putData("Arm Go To Zero", m_armevator.armGoToZeroFactory());
    SmartDashboard.putData("Stop Arm", m_armevator.stopArmFactory());
    SmartDashboard.putData("Go To One Inch", m_armevator.goToOneInchFactory());
    SmartDashboard.putData("L4", m_armevator.L4Factory());
    SmartDashboard.putData("L3", m_armevator.L3Factory());
    SmartDashboard.putData("L2", m_armevator.L2Factory());
    SmartDashboard.putData("L2 Algae", L2AlgaePickupFactory());
    SmartDashboard.putData("L3 Algae", L3AlgaePickupFactory());
    SmartDashboard.putData("Shoot In Net", shootInNet());
    SmartDashboard.putData("Fling In Net", netflingCommand());
    SmartDashboard.putData("Shoot Full Speed", m_doghouse.shootFullSpeedFactory());

    SmartDashboard.putData("Stop Doghouse", m_doghouse.stopAllFactory());
    SmartDashboard.putData("Shoot", m_doghouse.shootFactory(1.0));

    SmartDashboard.putData("Go To Tilt Angle", goToTiltAngleFactory());
    SmartDashboard.putData("Algae Pickup", algaePickupFactory());
    SmartDashboard.putData("Get Coral", getCoralFactory());

    // Climber
    SmartDashboard.putData("OpenGrabber", climber.openGrabberFactory());
    SmartDashboard.putData("SoftClose", climber.softCloseFactory());
    SmartDashboard.putData("Calibrate Gripper", climber.calibrateGripperFactory());
    SmartDashboard.putData("CloseGrabber", climber.closeGrabberFactory());
    SmartDashboard.putData("SetGasMotorRotations", climber.setGasMotorRotationsFactory());
    SmartDashboard.putData("StopGasMotor", climber.stopGasMotorFactory());
    // SmartDashboard.putData(
    //     "CalibrateGasMotor", climber.calibrateGasMotorFactory().ignoringDisable(true));
    SmartDashboard.putData("SetPositionInches", climber.setGasMotorInchesFactory());
    SmartDashboard.putData(
        "Calibrate Encoder", climber.calibrateEncoderFactory().ignoringDisable(true));
    SmartDashboard.putData("Set Coast", climber.gasMotorNeutralModeFactory().ignoringDisable(true));
    SmartDashboard.putData("Set Brake", climber.gasMotorBrakeModeFactory().ignoringDisable(true));
    SmartDashboard.putData("Prep Climber", climber.prepClimber());
    SmartDashboard.putData("Calibrate Gas Motor", climber.calibrateFactory());

    // Autos
    SmartDashboard.putData("Score 5", scoreAuto(5));
    SmartDashboard.putData("Score 6", scoreAuto(6));
    SmartDashboard.putData("Score 7", scoreAuto(7));
    SmartDashboard.putData("Score 8", scoreAuto(8));
    SmartDashboard.putData("Score 9", scoreAuto(9));
    SmartDashboard.putData("Score 10", scoreAuto(10));
    SmartDashboard.putData("Score 11", scoreAuto(11));
    SmartDashboard.putData("Score 12", scoreAuto(12));
    SmartDashboard.putData("Score 1", scoreAuto(1));
    SmartDashboard.putData("Score 2", scoreAuto(2));
    SmartDashboard.putData("Score 3", scoreAuto(3));
    SmartDashboard.putData("Score 4", scoreAuto(4));
    // SmartDashboard.putData("TripleL1Right", pathFinder.setGoalPosition(new
    // Translation2d(4.1148)));
  }

  public void configureButtonBox() {
    buttons.button(15).onTrue(new InstantCommand(() -> mode = 1));
    buttons.button(1).onTrue(new InstantCommand(() -> mode = 2));
    buttons.button(2).onTrue(new InstantCommand(() -> mode = 3));
    buttons.button(3).onTrue(new InstantCommand(() -> mode = 4));
    buttons.button(10).onTrue(shootCommand(1.0));
    buttons.button(4).onTrue(m_armevator.stowFactory());
    buttons.button(5).onTrue(getCoralFactory());
    buttons.button(11).onTrue(algaePickupFactory());
    buttons
        .button(7)
        .onTrue(
            climber
                .prepClimber()
                .alongWith(
                    m_doghouse.stopAllFactory(), new InstantCommand(() -> drive.disableAutoAim())));
    buttons.button(8).onTrue(L3AlgaePickupFactory());
    buttons.button(9).onTrue(L2AlgaePickupFactory());
    buttons.button(12).onTrue(shootInNet());
    buttons
        .button(13)
        .onTrue(
            shootInNet()
                .alongWith(
                    DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX())));
    buttons
        .button(6)
        .onTrue(climber.gasMotorNeutralModeFactory().andThen(climber.stopGasMotorFactory()));

    controller.rightTrigger().onTrue(shootCommand(1.0));
    controller
        .leftTrigger()
        .onTrue(m_doghouse.shootAlgaeFactory())
        .onFalse(drive.getDefaultCommand());
    controller
        .rightBumper()
        .onTrue(
            new ConditionalCommand(
                algae(), reef(true, 1.0), () -> !m_doghouse.hasCoral() && !m_doghouse.isBlocked()))
        .onFalse(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    m_armevator.AlgaestowFactory(),
                    DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX())),
                new ParallelCommandGroup(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () ->
                            Rotation2d.fromDegrees(
                                drive.aimAtExpectedTarget(() -> m_doghouse.hasCoral()))),
                    m_armevator.stowFactory()),
                () -> m_doghouse.isAlgaeMode()));
    controller
        .leftBumper()
        .onTrue(
            new ConditionalCommand(
                algae(), reef(false, 1.0), () -> !m_doghouse.hasCoral() && !m_doghouse.isBlocked()))
        .onFalse(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    m_armevator.AlgaestowFactory(),
                    DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX())),
                new ParallelCommandGroup(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () ->
                            Rotation2d.fromDegrees(
                                drive.aimAtExpectedTarget(() -> m_doghouse.hasCoral()))),
                    m_armevator.stowFactory()),
                () -> m_doghouse.isAlgaeMode()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_armevator.setDefaultCommand(m_armevator.defaultCommand());
    m_doghouse.setDefaultCommand(m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown()));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> Rotation2d.fromDegrees(drive.aimAtExpectedTarget(() -> m_doghouse.hasCoral()))));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed

    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    Rotation2d.fromDegrees(
                        (p_amplitude.getValue()
                                * Math.sin((p_frequency.getValue() * 2.0) * Math.PI * timer.get()))
                            + 90.0)));

    controller
        .y()
        .toggleOnTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  private Command getAutoPath(String pathName) {
    try {
      Command autoPath = new PathPlannerAuto(pathName);
      return autoPath;
    } catch (Exception e) {
      Command autoPath = new WaitCommand(1.0);
      System.err.println("Exception loading auto path");
      e.printStackTrace();
      return autoPath;
    }
  }

  private Command score(boolean odd) {
    return new SequentialCommandGroup(
        drive.pathFind(odd),
        new ParallelDeadlineGroup(drive.scoreOnReef(odd), m_armevator.scoreAll(() -> mode)),
        shootCommand(1.0));
  }

  private Command scoreOnReef(boolean odd) {
    return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                drive.pathFind(odd),
                m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())),
            new ParallelDeadlineGroup(
                new ConditionalCommand(
                    new WaitUntilCommand(
                        () ->
                            reefDebouncer.calculate(m_doghouse.getIsReefDetected())
                                && m_armevator.atMode(() -> mode)),
                    new WaitUntilCommand(
                        () -> drive.isShootingDistance() && m_armevator.atMode(() -> mode)),
                    () -> mode == 4),
                drive.scoreOnReef(odd),
                m_armevator.scoreAll(() -> mode),
                m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())),
            shootCommand(1.0))
        .beforeStarting(() -> reefDebouncer.calculate(false));
  }

  private Command scoreOnPath(boolean odd) {
    return new SequentialCommandGroup(
            drive.pathFind(odd),
            new ParallelDeadlineGroup(
                new WaitCommand(0.7), drive.scoreOnReef(odd), m_armevator.scoreAll(() -> mode)),
            shootCommand(1.0))
        .beforeStarting(() -> reefDebouncer.calculate(false));
  }

  private Command scoreNoShoot(boolean odd) {
    return new SequentialCommandGroup(
            drive.pathFind(odd),
            new ParallelDeadlineGroup(drive.scoreOnReef(odd), m_armevator.scoreAll(() -> mode)))
        .beforeStarting(() -> reefDebouncer.calculate(false));
  }

  private Command scoreAuto(int num) {
    try {
      PathPlannerPath autoPath = PathPlannerPath.fromPathFile("Score " + num);
      return AutoBuilder.pathfindThenFollowPath(autoPath, new PathConstraints(3.0, 3.0, 8.0, 20.0));
    } catch (Exception e) {
      Command autoPath = new WaitCommand(1.0);
      System.err.println("Exception loading auto path");
      e.printStackTrace();
      return autoPath;
    }
  }

  private Command shootCommand(double delay) {
    return new ParallelDeadlineGroup(
        new ConditionalCommand(
            m_doghouse.shootL1(), m_doghouse.shootFactory(delay), () -> mode == 1),
        new WaitCommand(0.15).andThen(m_armevator.stowFactory()),
        new InstantCommand(() -> drive.enableAutoAim()),
        new InstantCommand(() -> Logger.recordOutput("ShotPose", drive.getPose())));
  }

  private Command shoot() {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> m_doghouse.getIsReefDetected())
            .andThen(m_doghouse.shootFactory(1.0)),
        new WaitCommand(0.5).andThen(m_armevator.armGoToZeroFactory()),
        new InstantCommand(() -> Logger.recordOutput("ShotPose", drive.getPose())));
  }

  private Command netflingCommand() {
    return new ParallelDeadlineGroup(
            m_armevator.shootInNetFactory(),
            new WaitUntilCommand(
                    () ->
                        m_armevator.getElevatorPositionInches()
                            > (Constants.ELEVATOR_L4_HEIGHT - 6))
                .andThen(m_doghouse.shootFullSpeedFactory()))
        .andThen(m_armevator.stowFactory());
  }

  private Command algaePickupFactory() {
    return new ParallelCommandGroup(
        m_armevator
            .algaePickupFactory()
            .until(() -> m_doghouse.hasCoralDebounced())
            .andThen(m_armevator.AlgaestowFactory()),
        m_doghouse
            .setAlgaeModeFactory()
            .andThen(m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())));
  }

  private Command L2AlgaePickupFactory() {
    return new ParallelCommandGroup(
        m_armevator.L2Algae()
            .until(() -> m_doghouse.hasCoralDebounced())
            .andThen(m_armevator.AlgaestowFactory()),
        m_doghouse
            .setAlgaeModeFactory()
            .andThen(m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())));
  }

  private Command L3AlgaePickupFactory() {
    return new ParallelCommandGroup(
        m_armevator.L3Algae()
            .until(() -> m_doghouse.hasCoralDebounced())
            .andThen(m_armevator.AlgaestowFactory()),
        m_doghouse
            .setAlgaeModeFactory()
            .andThen(m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())));
  }

  public Command shootInNet() {
    // return m_armevator.shootInNetFactory();
    return new ParallelDeadlineGroup(
            new WaitCommand(0.5).andThen(m_doghouse.shootFullSpeedFactory()),
            m_armevator.shootInNetFactory())
        .andThen(m_armevator.stowFactory());
  }

  private Command goToTiltAngleFactory() {
    return new ParallelCommandGroup(
        m_armevator.goToTiltAngleFactory(), m_doghouse.stopAllFactory());
  }

  private Command getCoralFactory() {
    return new ParallelDeadlineGroup(
        m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown()),
        m_armevator.armGoToZeroFactory());
  }

  private boolean hasCoralDebounced() {
    return m_doghouse.hasCoralDebounced();
  }

  private Command algae() {
    return new ParallelCommandGroup(
        drive
            .algae()
            .andThen(
                DriveCommands.joystickDrive(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> -controller.getRightX())),
        m_armevator.algae(() -> drive.getTargetSectorNow()),
        m_doghouse
            .setAlgaeModeFactory()
            .andThen(m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())));
  }

  public Command shootAlgae() {
    return m_doghouse
        .shootAlgaeFactory()
        .andThen(m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown()))
        .alongWith(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    Rotation2d.fromDegrees(
                        drive.aimAtExpectedTarget(() -> m_doghouse.hasCoral()))));
  }

  private Command reef(boolean odd, double delay) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new ConditionalCommand(
                    new ParallelRaceGroup(
                        new WaitUntilCommand(
                            () ->
                                reefDebouncer.calculate(m_doghouse.getIsReefDetected())
                                    && m_armevator.atMode(() -> mode)
                                    && drive.isAtTarget()),
                        new WaitCommand(3.0)),
                    new WaitUntilCommand(
                        () -> m_armevator.atMode(() -> mode) && drive.isAtTarget()),
                    () -> mode == 4),
                new ConditionalCommand(
                    new ParallelRaceGroup(
                        new WaitUntilCommand(
                            () ->
                                reefDebouncer.calculate(m_doghouse.getIsReefDetected())
                                    && m_armevator.atMode(() -> mode)),
                        new WaitCommand(3.0)),
                    new WaitUntilCommand(
                        () -> m_armevator.atMode(() -> mode) && drive.isShootingDistance()),
                    () -> mode == 4),
                () -> drive.isElevatorDistance()),
            drive.reef(odd, () -> mode),
            new WaitUntilCommand(drive::isElevatorDistance)
                .andThen(m_armevator.scoreAll(() -> mode)),
            m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())),
        shootCommand(delay));
  }

  public Command reefAuto(boolean odd, double delay) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            drive.reef(odd, () -> mode),
            new WaitUntilCommand(drive::isElevatorDistance)
                .andThen(m_armevator.scoreAll(() -> mode))),
        shootCommand(delay));
  }

  private Command reefNoShoot(boolean odd) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new ConditionalCommand(
                    new WaitUntilCommand(
                        () ->
                            reefDebouncer.calculate(m_doghouse.getIsReefDetected())
                                && m_armevator.atMode(() -> mode)
                                && drive.isAtTarget()),
                    new WaitUntilCommand(
                        () -> m_armevator.atMode(() -> mode) && drive.isAtTarget()),
                    () -> mode == 4),
                new ConditionalCommand(
                    new WaitUntilCommand(
                        () ->
                            reefDebouncer.calculate(m_doghouse.getIsReefDetected())
                                && m_armevator.atMode(() -> mode)),
                    new WaitUntilCommand(
                        () -> m_armevator.atMode(() -> mode) && drive.isShootingDistance()),
                    () -> mode == 4),
                () -> drive.isElevatorDistance()),
            drive.reef(odd, () -> mode),
            new WaitUntilCommand(drive::isElevatorDistance)
                .andThen(m_armevator.scoreAll(() -> mode)),
            m_doghouse.coralIntakeFactory(() -> m_armevator.isElevatorDown())));
  }

  private Command reefMoving(boolean odd) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new WaitUntilCommand(
                    () ->
                        reefDebouncer.calculate(m_doghouse.getIsReefDetected())
                            && m_armevator.atMode(() -> mode)
                            && drive.isAtTarget()),
                new WaitUntilCommand(() -> m_armevator.atMode(() -> mode) && drive.isAtTarget()),
                () -> mode == 4),
            drive
                .reefMoving(odd)
                .andThen(DriveCommands.driveMoving(() -> 0.0, () -> 0.3, () -> 0.0, drive)),
            new WaitUntilCommand(drive::isElevatorDistance)
                .andThen(m_armevator.scoreAll(() -> mode))),
        shootCommand(1.0));
  }

  public void teleopInit() {
    new SequentialCommandGroup(climber.calibrateFactory(), climber.calibrateGripperFactory())
        .schedule();
  }

  public void disableInit() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
