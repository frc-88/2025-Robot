package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Armevator extends SubsystemBase {
  private TalonFX m_elevatorMain = new TalonFX(Constants.ELEVATOR_MAIN_MOTOR, Constants.RIO_CANBUS);
  private TalonFX m_elevatorFollower =
      new TalonFX(Constants.ELEVATOR_FOLLOWER_MOTOR, Constants.RIO_CANBUS);
  private TalonFX m_arm = new TalonFX(Constants.ELEVATOR_ARM_MOTOR, Constants.RIO_CANBUS);
  private TalonFX m_manipulator =
      new TalonFX(Constants.ELEVATOR_MANIPULATOR_MOTOR, Constants.RIO_CANBUS);

  private final CANrange m_doghousCANRange =
      new CANrange(Constants.DOGHOUSE_CANRANGE, Constants.RIO_CANBUS);
  private final CANrange m_coralRange =
      new CANrange(Constants.CORAL_CANRANGE, Constants.RIO_CANBUS);
  // private final CANrange m_canRangeRight =
  //     new CANrange(Constants.ARM_RIGHT_CANRANGE, Constants.RIO_CANBUS);

  private final Debouncer elevatorDebouncer = new Debouncer(1.0);

  private PIDPreferenceConstants elevatorPID =
      new PIDPreferenceConstants("Armevator/Elevator/PID", 8, 0, 0, 0.15, 0, 0, 0, 0);
  private DoublePreferenceConstant p_elevatorMaxVelocity =
      new DoublePreferenceConstant("Armevator/Elevator/MotionMagicVelocity", 60.0);
  private DoublePreferenceConstant p_elevatorMaxAcceleration =
      new DoublePreferenceConstant("Armevator/Elevator/MotionMagicAcceleration", 120.0);
  private DoublePreferenceConstant p_elevatorJerk =
      new DoublePreferenceConstant("Armevator/Elevator/MotionMagicJerk", 1200.0);
  private DoublePreferenceConstant p_elevatorTargetInches =
      new DoublePreferenceConstant("Armevator/Elevator/TargetPositionInches", 6.0);

  private PIDPreferenceConstants armPID =
      new PIDPreferenceConstants("Armevator/Arm/PID", 1, 0, 0, 0.12, 0, 0, 0, 0);
  private DoublePreferenceConstant p_armMaxVelocity =
      new DoublePreferenceConstant("Armevator/Arm/MotionMagicVelocity", 40.0);
  private DoublePreferenceConstant p_armMaxAcceleration =
      new DoublePreferenceConstant("Armevator/Arm/MotionMagicAcceleration", 80.0);
  private DoublePreferenceConstant p_armJerk =
      new DoublePreferenceConstant("Armevator/Arm/MotionMagicJerk", 0.0);
  private DoublePreferenceConstant p_armTargetDegrees =
      new DoublePreferenceConstant("Armevator/Arm/TargetPositionDegrees", 45.0);
  private DoublePreferenceConstant p_armTiltAngle =
      new DoublePreferenceConstant("Armevator/Arm/TiltAngle", 7.5);

  private DoublePreferenceConstant p_manipulatorInSpeed =
      new DoublePreferenceConstant("Armevator/Manipultor/InSpeed", 0.2);
  private DoublePreferenceConstant p_manipulatorOutSpeed =
      new DoublePreferenceConstant("Armevator/Manipultor/OutSpeed", 0.3);
  private DoublePreferenceConstant p_manipulatorCurrentLimit =
      new DoublePreferenceConstant("Armevator/Manipultor/CurrentLimit", 120);

  private MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);

  private boolean m_calibrated = false;
  private boolean hasCoral = false;
  private boolean isIn = false;

  public Trigger hasCoralTrigger = new Trigger(() -> hasCoral);
  public Trigger isInTrigger = new Trigger(() -> isIn);

  public Armevator() {
    configureTalons();
    configureCANrange();
  }

  private void configureTalons() {
    TalonFXConfiguration maincfg = new TalonFXConfiguration();
    TalonFXConfiguration followercfg = new TalonFXConfiguration();
    TalonFXConfiguration armcfg = new TalonFXConfiguration();

    TalonFXConfiguration manipulatorConfiguration = new TalonFXConfiguration();
    manipulatorConfiguration.CurrentLimits.SupplyCurrentLimit =
        p_manipulatorCurrentLimit.getValue();
    manipulatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // manipulatorConfiguration.OpenLoopRamps =
    //    new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0);
    m_manipulator.getConfigurator().apply(manipulatorConfiguration);

    maincfg.Slot0.kP = elevatorPID.getKP().getValue();
    maincfg.Slot0.kI = elevatorPID.getKI().getValue();
    maincfg.Slot0.kD = elevatorPID.getKD().getValue();
    maincfg.Slot0.kV = elevatorPID.getKF().getValue();

    maincfg.MotionMagic.MotionMagicCruiseVelocity = p_elevatorMaxVelocity.getValue();
    maincfg.MotionMagic.MotionMagicAcceleration = p_elevatorMaxAcceleration.getValue();
    maincfg.MotionMagic.MotionMagicJerk = p_elevatorJerk.getValue();

    armcfg.Slot0.kP = armPID.getKP().getValue();
    armcfg.Slot0.kI = armPID.getKI().getValue();
    armcfg.Slot0.kD = armPID.getKD().getValue();
    armcfg.Slot0.kV = armPID.getKF().getValue();

    armcfg.MotionMagic.MotionMagicCruiseVelocity = p_armMaxVelocity.getValue();
    armcfg.MotionMagic.MotionMagicAcceleration = p_armMaxAcceleration.getValue();
    armcfg.MotionMagic.MotionMagicJerk = p_armJerk.getValue();

    m_elevatorMain.getConfigurator().apply(maincfg);
    m_elevatorFollower.getConfigurator().apply(followercfg);
    m_arm.getConfigurator().apply(armcfg);

    m_arm.setNeutralMode(NeutralModeValue.Brake);

    m_elevatorFollower.setControl(new Follower(Constants.ELEVATOR_MAIN_MOTOR, false));
  }

  private void configureCANrange() {
    CANrangeConfiguration coralRangecfg = new CANrangeConfiguration();
    CANrangeConfiguration doghouscfg = new CANrangeConfiguration();
    // CANrangeConfiguration canRangerightcfg = new CANrangeConfiguration();
    coralRangecfg.FovParams.FOVRangeX = 6.75;
    doghouscfg.FovParams.FOVRangeX = 7.5;
    coralRangecfg.FovParams.FOVRangeY = 6.75;
    doghouscfg.FovParams.FOVRangeY = 27.0;
    // canRangerightcfg.FovParams.FOVRangeY = 27.0;

    doghouscfg.ToFParams.UpdateFrequency = 50;
    coralRangecfg.ToFParams.UpdateFrequency = 50;

    doghouscfg.ProximityParams.ProximityThreshold = 0.5;
    doghouscfg.ProximityParams.ProximityHysteresis = 0.03;
    coralRangecfg.ProximityParams.ProximityThreshold = 0.5;

    coralRangecfg.ProximityParams.MinSignalStrengthForValidMeasurement = 40000;
    doghouscfg.ProximityParams.MinSignalStrengthForValidMeasurement = 10000;

    m_doghousCANRange.getConfigurator().apply(doghouscfg);
    // m_canRangeRight.getConfigurator().apply(canRangerightcfg);
    m_coralRange.getConfigurator().apply(coralRangecfg);
  }

  public boolean isCoralDetected() {
    return m_coralRange.getIsDetected().getValue();
  }

  public boolean isDoghouseDetected() {
    return m_doghousCANRange.getIsDetected().getValue();
  }

  public double getArmAngle() {
    return m_arm.getPosition().getValueAsDouble() * Constants.ARM_ROTATIONS_TO_DEGREES;
  }

  private void armCalibrate() {
    m_arm.setPosition(0.0);
  }

  private void elevatorCalibrate() {
    m_elevatorMain.setPosition(0.0);
  }

  public Trigger hasCoral() {
    return hasCoralTrigger;
  }

  public double getElevatorPositionInches() {
    return m_elevatorMain.getPosition().getValueAsDouble() * Constants.ELEVATOR_ROTATIONS_TO_INCHES;
  }

  private void elevatorSetPosition(double position) {
    m_elevatorMain.setControl(
        motionmagicrequest
            .withPosition(position / Constants.ELEVATOR_ROTATIONS_TO_INCHES)
            .withFeedForward(0.056));
  }

  private void armSetAngle(double angle) {
    m_arm.setControl(motionmagicrequest.withPosition(angle / Constants.ARM_ROTATIONS_TO_DEGREES));
  }

  private void armGoToTiltAngle() {
    armSetAngle(p_armTiltAngle.getValue());
  }

  private void armGotoAlgaePickup() {
    armSetAngle(56.0);
  }

  private void armGotoPrefPosition() {
    armSetAngle(p_armTargetDegrees.getValue());
  }

  private void armGoToZero() {
    armSetAngle(0.0);
  }

  private void setL4() {
    elevatorSetPosition(Constants.ELEVATOR_L4_HEIGHT);
    if (getElevatorPositionInches() > (Constants.ELEVATOR_L4_HEIGHT - 2)) {
      armSetAngle(Constants.ARM_L4_ANGLE);
    }
  }

  private void setL3() {
    elevatorSetPosition(Constants.ELEVATOR_L3_HEIGHT);
    armGoToTiltAngle();
  }

  private void setL2() {
    elevatorSetPosition(Constants.ELEVATOR_L2_HEIGHT);
    armGoToTiltAngle();
  }

  private void elevatorStop() {
    m_elevatorMain.setControl(new DutyCycleOut(0.0));
  }

  private void armStop() {
    m_arm.setControl(new DutyCycleOut(0.0));
  }

  private void manipulatorStop() {
    m_manipulator.setControl(new DutyCycleOut(0.0));
  }

  private void manipulatorIn() {
    m_manipulator.setControl(new DutyCycleOut(p_manipulatorInSpeed.getValue()));
  }

  private void manipulatorOut() {
    m_manipulator.setControl(new DutyCycleOut(p_manipulatorOutSpeed.getValue()));
  }

  private void manipulatorReverse() {
    m_manipulator.setControl(new DutyCycleOut(0.1));
  }

  private void manipulatorSlow() {
    m_manipulator.setControl(new DutyCycleOut(-0.1));
  }

  public boolean isArmZero() {
    return Math.abs(getArmAngle()) < 1.2;
  }

  public boolean isArmOnPosition() {
    return Math.abs(getArmAngle() - 7.5) < 1.2;
  }

  private void elevatorSetSlowSpeed() {
    m_elevatorMain.setControl(new DutyCycleOut(0.1));
  }

  private void armSetSlowSpeed() {
    m_arm.setControl(new DutyCycleOut(0.1));
  }

  private void elevatorSetCalibrateSpeed() {
    m_elevatorMain.setControl(new DutyCycleOut(-0.09));
  }

  private void stowArm() {
    armGoToTiltAngle();
  }

  private void stowElevator() {
    elevatorSetPosition(0.0);
  }

  public void stowBoth() {
    stowArm();
    stowElevator();
  }

  public Trigger isIn() {
    return isInTrigger;
  }

  public Command stowArmFactory() {
    return new RunCommand(() -> stowArm(), this);
  }

  public Command stowBothFactory() {
    return new RunCommand(() -> stowArmFactory(), this);
  }

  public Command calibrateArmFactory() {
    return new InstantCommand(() -> armCalibrate(), this);
  }

  public Command calibrateElevatorFactory() {
    return new RunCommand(
            () -> {
              stowElevator();
              stowArm();
            },
            this)
        .until(
            () ->
                elevatorDebouncer.calculate(
                        Math.abs(m_elevatorMain.getVelocity().getValueAsDouble()) < 0.02)
                    && getElevatorPositionInches() < 1.0)
        .andThen(
            () -> {
              elevatorStop();
              stowArm();
            })
        .andThen(new WaitCommand(0.25))
        .andThen((() -> elevatorCalibrate()))
        .andThen(
            new RunCommand(
                () -> {
                  stowElevator();
                  stowArm();
                }));
  }

  public Command calibrateBothFactory() {
    return new InstantCommand(() -> armCalibrate(), this)
        .andThen(
            () -> {
              elevatorSetCalibrateSpeed();
              armGoToZero();
            })
        .until(
            () ->
                elevatorDebouncer.calculate(
                    Math.abs(m_elevatorMain.getVelocity().getValueAsDouble()) < 0.02))
        .andThen(() -> elevatorStop())
        .andThen(new WaitCommand(0.25))
        .andThen(
            () -> {
              elevatorCalibrate();
              m_calibrated = true;
            })
        .beforeStarting(() -> elevatorDebouncer.calculate(false));
  }

  public Command manipulatorOutFactory() {
    return new RunCommand(() -> manipulatorOut(), this)
        .withTimeout(1.0)
        .andThen(
            () -> {
              manipulatorStop();
              hasCoral = false;
              isIn = false;
            });
  }

  public Command backUpFactory() {
    return new RunCommand(() -> manipulatorReverse(), this)
        .until(() -> !m_coralRange.getIsDetected().getValue())
        .andThen(
            () -> {
              manipulatorStop();
              isIn = true;
            });
  }

  public Command manipulatorInFactory() {
    return new RunCommand(
        () -> {
          if (!m_coralRange.getIsDetected().getValue()
              && !m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorIn();
          } else if (m_coralRange.getIsDetected().getValue()
              && m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorSlow();
          } else if (m_coralRange.getIsDetected().getValue()
              && !m_doghousCANRange.getIsDetected().getValue()) {
            manipulatorStop();
          }
          armGoToZero();
        },
        this);
  }

  public Command algaePickupFactory() {
    return new RunCommand(
        () -> {
          armGotoAlgaePickup();
          m_manipulator.setControl(new DutyCycleOut(1.0));
        },
        this);
  }

  public Command goToOneInchFactory() {
    return new RunCommand(() -> elevatorSetPosition(1.0), this);
  }

  public Command manipulatorStopFactory() {
    return new InstantCommand(() -> manipulatorStop(), this);
  }

  public Command stopElevatorFactory() {
    return new RunCommand(() -> elevatorStop(), this);
  }

  public Command setElevatorSlowSpeedFactory() {
    return new RunCommand(() -> elevatorSetSlowSpeed(), this);
  }

  public Command stowFactory() {
    return new SequentialCommandGroup(
        stowArmFactory().until(this::isArmOnPosition), calibrateElevatorFactory());
  }

  public Command goToTiltAngleFactory() {
    return new RunCommand(
            () -> {
              armGoToTiltAngle();
              manipulatorStop();
            },
            this)
        .until(this::isArmOnPosition);
  }

  public Command armGoToZeroFactory() {
    return new RunCommand(() -> armGoToZero(), this);
  }

  public Command setElevatorPostionFactory() {
    return new RunCommand(() -> elevatorSetPosition(p_elevatorTargetInches.getValue()), this);
  }

  public Command stopArmFactory() {
    return new RunCommand(() -> armStop(), this);
  }

  public Command setArmSlowSpeedFactory() {
    return new RunCommand(() -> armSetSlowSpeed(), this);
  }

  public Command setArmPostionFactory() {
    return new RunCommand(() -> armGotoPrefPosition(), this);
  }

  public Command defaultCommand() {
    return new ConditionalCommand(stowFactory(), calibrateBothFactory(), () -> m_calibrated);
  }

  public Command L4Factory() {
    return new RunCommand(() -> setL4(), this);
  }

  public Command L3Factory() {
    return new RunCommand(() -> setL3(), this);
  }

  public Command L2Factory() {
    return new RunCommand(() -> setL2(), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Positon", getElevatorPositionInches());
    SmartDashboard.putBoolean("Is detected", m_coralRange.getIsDetected().getValue());
    SmartDashboard.putNumber("Arm Position", getArmAngle());
    SmartDashboard.putNumber(
        "CAN Range Left Distance",
        Units.metersToInches(m_doghousCANRange.getDistance().getValueAsDouble()));
    SmartDashboard.putNumber(
        "CAN Range Middle Distance",
        Units.metersToInches(m_coralRange.getDistance().getValueAsDouble()));
    SmartDashboard.putBoolean("Doghouse CANRange", m_doghousCANRange.getIsDetected().getValue());
    // SmartDashboard.putBoolean("Right CAN Range", m_canRangeRight.getIsDetected().getValue());
    SmartDashboard.putBoolean("Coral CANRange", m_coralRange.getIsDetected().getValue());
  }
}
