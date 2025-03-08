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
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
  Timer timer = new Timer();
  private DoublePreferenceConstant p_amplitude = new DoublePreferenceConstant("Aplitude", 0);
  private DoublePreferenceConstant p_frequency = new DoublePreferenceConstant("Wavelength", 0);

  // public Trigger atL4 = new Trigger(() -> hasCoralDebounced() && m_armevator.atL4());
  // public Trigger atL3 = new Trigger(() -> hasCoralDebounced() && m_armevator.atL3());
  // public Trigger atL2 =
  //     new Trigger(
  //         () -> hasCoralDebounced() && m_armevator.atL2() && m_doghouse.getIsReefDetected());
  public int mode = 2;

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

    registerNamedCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // ready functions replaced with lambdas returning true for now...need to be implemented in each
    // subsystem
    lights =
        new Lights(
            () -> true,
            () -> true,
            () -> true,
            () -> true,
            () -> true,
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
    NamedCommands.registerCommand("Shoot", shootCommand());
    NamedCommands.registerCommand("NetFling", netflingCommand());
    NamedCommands.registerCommand("Get Coral", getCoralFactory());
    NamedCommands.registerCommand(
        "Arm Go To Zero", m_armevator.armGoToZeroFactory().withTimeout(0.5));
    NamedCommands.registerCommand("L4", m_armevator.L4Factory().withTimeout(2.0));
    NamedCommands.registerCommand("Stow", m_armevator.stowFactory().withTimeout(1.0));
    NamedCommands.registerCommand("Armevator Calibration", m_armevator.calibrateBothFactory());
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
                                () -> controller.setRumble(RumbleType.kBothRumble, 0)))));

    climber.shouldSoftCloseTrigger.onTrue(climber.softCloseFactory());
    m_armevator.m_shouldCalibrate.onTrue(m_armevator.elevatorCalibrateFactory());
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
    SmartDashboard.putData("Shoot", m_doghouse.shootFactory());

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
    SmartDashboard.putData("TripleL1Right", getAutoPath("TripleL1Right"));
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
    buttons.button(1).onTrue(new InstantCommand(() -> mode = 2));
    buttons.button(2).onTrue(new InstantCommand(() -> mode = 3));
    buttons.button(3).onTrue(new InstantCommand(() -> mode = 4));
    buttons.button(10).onTrue(shootCommand());
    buttons.button(4).onTrue(m_armevator.stowFactory());
    buttons.button(5).onTrue(getCoralFactory());
    buttons.button(11).onTrue(algaePickupFactory());
    buttons.button(7).onTrue(climber.prepClimber());
    buttons.button(8).onTrue(L3AlgaePickupFactory());
    buttons.button(9).onTrue(L2AlgaePickupFactory());
    buttons.button(12).onTrue(m_armevator.shootInNetFactory());
    buttons.button(13).onTrue(netflingCommand());

    controller.rightTrigger().onTrue(shootCommand());
    controller.rightBumper().onTrue(score(true)).onFalse(drive.getDefaultCommand());
    controller.leftBumper().onTrue(score(false)).onFalse(drive.getDefaultCommand());
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
        .onTrue(
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
        shootCommand());
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

  private Command shootCommand() {
    return new ParallelDeadlineGroup(
        m_doghouse.shootFactory(),
        new WaitCommand(0.5).andThen(m_armevator.armGoToZeroFactory()),
        new InstantCommand(() -> Logger.recordOutput("ShotPose", drive.getPose())));
  }

  private Command shoot() {
    return new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> m_doghouse.getIsReefDetected())
            .andThen(m_doghouse.shootFactory()),
        new WaitCommand(0.5).andThen(m_armevator.armGoToZeroFactory()),
        new InstantCommand(() -> Logger.recordOutput("ShotPose", drive.getPose())));
  }

  private Command netflingCommand() {
    return new ParallelDeadlineGroup(
            new WaitCommand(0.15).andThen(m_doghouse.shootFullSpeedFactory()),
            m_armevator.armGoToZeroFactory())
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
    return new ParallelCommandGroup(
        m_armevator.shootInNetFactory(), m_doghouse.setAlgaeSlowFactory());
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
