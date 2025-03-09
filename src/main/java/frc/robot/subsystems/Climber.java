// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Climber extends SubsystemBase {
  private DoublePreferenceConstant p_grippermaxVelocity =
      new DoublePreferenceConstant("Climber/Gripper/MotionMagicVelocity", 40);
  private DoublePreferenceConstant p_grippermaxAcceleration =
      new DoublePreferenceConstant("Climber/Gripper/MotionMagicAcceleration", 1000);
  private DoublePreferenceConstant p_grippermaxJerk =
      new DoublePreferenceConstant("Climber/Gripper/MotionMagicJerk", 0);
  private PIDPreferenceConstants p_gripperPidPreferenceConstants =
      new PIDPreferenceConstants("Climber/Gripper/PID", 3.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0);
  private DoublePreferenceConstant p_gripperLimit =
      new DoublePreferenceConstant("Climber/Gripper/Limit", 100);
  private DoublePreferenceConstant p_gripperPosition =
      new DoublePreferenceConstant("Climber/Gripper/MotorAngle", 90.0);
  private DoublePreferenceConstant p_gripperStowSpeed =
      new DoublePreferenceConstant("Climber/Gripper/StowSpeed", 0.01);

  private PIDPreferenceConstants p_gasmotorPID =
      new PIDPreferenceConstants("Climber/GasMotor/PID", 0.1, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0);
  private DoublePreferenceConstant p_gasmotorLimit =
      new DoublePreferenceConstant("Climber/GasMotor/Limit", 10.0);
  private DoublePreferenceConstant p_gasmotorPositionInches =
      new DoublePreferenceConstant("Climber/GasMotor/PositionInches", 7.0);
  private DoublePreferenceConstant p_gasmotorPositionRotations =
      new DoublePreferenceConstant("Climber/GasMotor/PositionRotations", 184.0);
  private DoublePreferenceConstant p_gripperSoftClose =
      new DoublePreferenceConstant("Climber/Gripper/GripperSoftCloseAngle", 10);

  private final TalonFX m_gripper =
      new TalonFX(Constants.CLIMBER_GRIPPER_MOTOR, Constants.RIO_CANBUS);
  private final TalonFX m_gasmotor = new TalonFX(Constants.CLIMBER_GAS_MOTOR, Constants.RIO_CANBUS);
  private final CANcoder m_climberEncoder =
      new CANcoder(Constants.CLIMBER_ENCODER, Constants.RIO_CANBUS);
  private final CANrange m_canRange =
      new CANrange(Constants.CLIMBER_CANRANGE, Constants.RIO_CANBUS);

  private DigitalInput input = new DigitalInput(9);
  private boolean isCalibrated = false;
  private boolean m_grabbed = false;
  private double kGripperMotorRotationsToAngle = Constants.GRIPPER_MOTOR_ROTATIONS_TO_ANGLE;

  private MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0);
  private PositionDutyCycle position = new PositionDutyCycle(0.0);
  private boolean isClimbing = false;
  private boolean m_calibrated = false;
  VisionIOLimelight vision = new VisionIOLimelight("", () -> new Rotation2d());

  private Debouncer climberDebouncer = new Debouncer(1.0);
  private Debouncer gripperDebouncer = new Debouncer(0.1);
  public Trigger onDisable = new Trigger(() -> shouldEnableNeutralOnDisable());
  public Trigger shouldCloseTrigger = new Trigger(() -> shouldClose() && RobotState.isEnabled());
  public Trigger shouldSoftCloseTrigger =
      new Trigger(() -> shouldSoftClose() && RobotState.isEnabled());
  // public Trigger forceCloseTrigger = new Trigger(() -> forceClose());

  /** Creates a new Climber. */
  public Climber() {
    configureTalons();
    calibrateBoth();
  }

  private void configureTalons() {
    TalonFXConfiguration grippercfg = new TalonFXConfiguration();
    TalonFXConfiguration gasmotorcfg = new TalonFXConfiguration();

    MotionMagicConfigs gripper_mm = grippercfg.MotionMagic;

    gripper_mm.MotionMagicCruiseVelocity = p_grippermaxVelocity.getValue();
    gripper_mm.MotionMagicAcceleration = p_grippermaxAcceleration.getValue();
    gripper_mm.MotionMagicJerk = p_grippermaxJerk.getValue();

    Slot0Configs gripperslot0 = grippercfg.Slot0;
    Slot0Configs gasmotorslot0 = gasmotorcfg.Slot0;

    gripperslot0.kP = p_gripperPidPreferenceConstants.getKP().getValue();
    gripperslot0.kI = p_gripperPidPreferenceConstants.getKI().getValue();
    gripperslot0.kD = p_gripperPidPreferenceConstants.getKD().getValue();
    gripperslot0.kV = p_gripperPidPreferenceConstants.getKF().getValue();
    gripperslot0.kS =
        p_gripperPidPreferenceConstants
            .getKS()
            .getValue(); // Approximately 0.25V to get the mechanism moving

    gasmotorslot0.kP = p_gasmotorPID.getKP().getValue();
    gasmotorslot0.kI = p_gasmotorPID.getKI().getValue();
    gasmotorslot0.kD = p_gasmotorPID.getKD().getValue();
    gasmotorslot0.kV = p_gasmotorPID.getKF().getValue();
    gasmotorslot0.kS = p_gasmotorPID.getKS().getValue();

    SoftwareLimitSwitchConfigs gripperSoftLimits = grippercfg.SoftwareLimitSwitch;
    gripperSoftLimits.ForwardSoftLimitEnable = false;
    gripperSoftLimits.ForwardSoftLimitThreshold = p_gripperLimit.getValue();

    SoftwareLimitSwitchConfigs gasmotorsoftlimtis = gasmotorcfg.SoftwareLimitSwitch;
    gasmotorsoftlimtis.ForwardSoftLimitEnable = true;
    gasmotorsoftlimtis.ForwardSoftLimitThreshold =
        (p_gasmotorLimit.getValue() / Constants.GAS_MOTOR_ROTATIONS_TO_LENGTH);
    gasmotorcfg.CurrentLimits.StatorCurrentLimitEnable = true;
    gasmotorcfg.CurrentLimits.StatorCurrentLimit = 75.0;
    gasmotorcfg.CurrentLimits.SupplyCurrentLimitEnable = false;
    grippercfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_gripper.getConfigurator().apply(grippercfg);

    m_gasmotor.getConfigurator().apply(gasmotorcfg);

    m_gasmotor.setNeutralMode(NeutralModeValue.Brake);

    m_gripper.setNeutralMode(NeutralModeValue.Brake);
  }

  public boolean isReady() {
    return m_gasmotor.isConnected()
        && m_gripper.isConnected()
        && m_climberEncoder.isConnected()
        && m_canRange.isConnected();
  }

  public Trigger shouldGripperClose() {
    return shouldCloseTrigger;
  }

  public Trigger shouldNeutral() {
    return onDisable;
  }

  public double getGasMotorRotationsFromInches(double inches) {
    return (inches / Constants.GAS_MOTOR_ROTATIONS_TO_LENGTH) * 1.21;
  }

  public double getGripperPositionRotations() {
    return m_gripper.getPosition().getValueAsDouble();
  }

  public boolean shouldClose() {
    return !input.get()
        && gripperDebouncer.calculate(
            m_canRange.getDistance().getValueAsDouble() > 0.19
                && m_canRange.getDistance().getValueAsDouble() < 0.20);
  }

  public boolean shouldSoftClose() {
    return !input.get() && m_canRange.getDistance().getValueAsDouble() > 0.24;
  }

  public boolean forceClose() {
    return RobotState.isDisabled() && !m_grabbed;
  }

  // public Trigger forceCloseOnDisable() {
  //   return forceCloseTrigger;
  // }

  public boolean onTarget() {
    return Math.abs(
            getPositionGasMotorRotations()
                - getGasMotorRotationsFromInches(p_gasmotorPositionInches.getValue()))
        < 1.0;
  }

  public boolean isStopped() {
    return Math.abs(getGasMotorVelocity()) < 1.0;
  }

  public boolean isGripperZero() {
    return Math.abs(getGripperPositionRotations()) < 3.0;
  }

  public double getAngleOfClimber() {
    return m_climberEncoder.getPosition().getValueAsDouble()
        * Constants.CLIMBER_ENCODER_ROTATIONS_TO_ANGLE;
  }

  public double getPositionGasMotorRotations() {
    return m_gasmotor.getPosition().getValueAsDouble();
  }

  public double getGasMotorVelocity() {
    return m_gasmotor.getVelocity().getValueAsDouble();
  }

  public boolean shouldEnableNeutralOnDisable() {

    return RobotState.isDisabled();
  }

  public boolean poweredClimb() {
    return getPositionGasMotorRotations() < 40.0;
  }

  private void gasMotorBrakeMode() {
    m_gasmotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private void gasMotorNeutralMode() {
    m_gasmotor.setNeutralMode(NeutralModeValue.Coast);
  }

  private void openGrabber() {
    setGripperAngle(p_gripperPosition.getValue());
    m_grabbed = false;
  }

  private void closeGrabber() {
    setGripperAngle(0.0);
  }

  private void stow() {
    if (isCalibrated) {
      m_gripper.setControl(m_motionMagic.withPosition(0.0));
    } else {
      m_gripper.setControl(new DutyCycleOut(-p_gripperStowSpeed.getValue()));

      if (climberDebouncer.calculate(m_gripper.getVelocity().getValueAsDouble() > -1)) {
        calibrateBoth();
        isCalibrated = true;
      }
    }
  }

  private void setGasMotorPostionInches(double inches) {
    isClimbing = true;
    m_gasmotor.setControl(position.withPosition(getGasMotorRotationsFromInches(inches)));
  }

  private void setGasMotorPositionRotations(double rotations) {
    m_gasmotor.setControl(position.withPosition(rotations));
  }

  private void setGripperAngle(double position) {
    m_gripper.setControl(m_motionMagic.withPosition(position / kGripperMotorRotationsToAngle));
  }

  private void holdPostion(double inches) {
    m_gasmotor.setControl(new PositionDutyCycle(getGasMotorRotationsFromInches(inches)));
  }

  public void setGasMotorCalibrateSpeed() {
    m_gasmotor.setControl(new DutyCycleOut(0.1));
  }

  private void stopGasMotor() {
    m_gasmotor.setControl(new DutyCycleOut(0.0));
  }

  private void calibrateGasMotor() {
    m_gasmotor.setPosition(0.0);
  }

  private void calibrateBoth() {
    m_gripper.setPosition(0.0);
    m_gasmotor.setPosition(0.0);
  }

  private void calibrateEncoder() {
    m_climberEncoder.setPosition(0.0);
  }

  private void setGripperCalibrateSpeed() {
    m_gripper.setControl(new DutyCycleOut(-0.2));
  }

  private void stopGripper() {
    m_gripper.setControl(new DutyCycleOut(0.0));
  }

  public Command softCloseFactory() {
    return new RunCommand(
        () -> {
          setGripperAngle(p_gripperSoftClose.getValue());
        },
        this);
  }

  public Command calibrateFactory() {
    return new InstantCommand(() -> gasMotorNeutralMode(), this)
        .andThen(new WaitCommand(1.0))
        .andThen(() -> gasMotorBrakeMode())
        .andThen(new WaitCommand(1.0))
        .andThen(
            new RunCommand(() -> setGasMotorCalibrateSpeed())
                .until(() -> m_gasmotor.getStatorCurrent().getValueAsDouble() > 3.0))
        .andThen(() -> stopGasMotor())
        .andThen(new WaitCommand(1.0))
        .andThen(() -> calibrateGasMotor())
        .beforeStarting(() -> climberDebouncer.calculate(false));
  }

  public Command stowFactory() {
    return new RunCommand(() -> stow(), this)
        .beforeStarting(() -> climberDebouncer.calculate(false));
  }

  public Command holdPositionFactory() {
    return new RunCommand(() -> holdPostion(p_gasmotorPositionInches.getValue()), this);
  }

  public Command openGrabberFactory() {
    return new RunCommand(() -> openGrabber(), this);
  }

  public Command setNotGrabbed() {
    return new InstantCommand(() -> m_grabbed = false);
  }

  public Command closeGrabberFactory() {
    return new RunCommand(
        () -> {
          closeGrabber();
          m_grabbed = true;
        },
        this);
  }

  public Command setGasMotorInchesFactory() {
    return new RunCommand(
        () -> setGasMotorPostionInches(p_gasmotorPositionInches.getValue()), this);
  }

  public Command setGasMotorRotationsFactory() {
    return new RunCommand(
        () -> setGasMotorPositionRotations(p_gasmotorPositionRotations.getValue()), this);
  }

  public Command stopGasMotorFactory() {
    return new RunCommand(() -> stopGasMotor(), this);
  }

  public Command gasMotorBrakeModeFactory() {
    return new InstantCommand(() -> gasMotorBrakeMode(), this);
  }

  public Command gasMotorNeutralModeFactory() {
    return new InstantCommand(() -> gasMotorNeutralMode(), this);
  }

  public Command gripperMotorNeutralModeFactory() {
    return new InstantCommand(() -> m_gripper.setNeutralMode(NeutralModeValue.Coast), this);
  }

  public Command gripperMotorBrakeModeFactory() {
    return new InstantCommand(() -> m_gripper.setNeutralMode(NeutralModeValue.Brake), this);
  }

  public Command calibrateGasMotorFactory() {
    return new InstantCommand(() -> calibrateGasMotor(), this);
  }

  public Command calibrateEncoderFactory() {
    return new InstantCommand(() -> calibrateEncoder(), this);
  }

  public Command calibrateGripperFactory() {
    return new RunCommand(() -> setGripperCalibrateSpeed(), this)
        .until(() -> m_gripper.getStatorCurrent().getValueAsDouble() > 10.0)
        .andThen(() -> m_gripper.setPosition(0.0))
        .andThen(() -> stopGripper());
  }

  public Command poweredClimbFactory() {
    return gasMotorNeutralModeFactory()
        .andThen(new WaitUntilCommand(() -> poweredClimb()))
        .andThen(gasMotorBrakeModeFactory());
  }

  public Command climbOnDisable() {
    return new InstantCommand(() -> m_gripper.setNeutralMode(NeutralModeValue.Coast))
        .andThen(new WaitCommand(1.0))
        .andThen(
            () -> {
              m_gripper.setNeutralMode(NeutralModeValue.Brake);
              m_grabbed = true;
            });
  }

  public Command prepClimber() {
    return new RunCommand(
        () -> {
          setGasMotorPositionRotations(p_gasmotorPositionRotations.getValue());
          openGrabber();
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler runn s
    SmartDashboard.putNumber("gas motor position rotations", getPositionGasMotorRotations());
    SmartDashboard.putBoolean(
        "Drive Forward", !input.get() && m_canRange.getDistance().getValueAsDouble() > 0.24);
    SmartDashboard.putNumber("Encoder position", m_climberEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Angle", getAngleOfClimber());
    SmartDashboard.putNumber(
        "Climber CAN Range Distance", m_canRange.getDistance().getValueAsDouble() * 100.0);
    SmartDashboard.putNumber("Gripper Position", getGripperPositionRotations());
    SmartDashboard.putBoolean("Sensor ouput", input.get());
    SmartDashboard.putBoolean("Grabbed", m_grabbed);
    SmartDashboard.putBoolean(
        "Is braked", m_gripper.getControlMode().getValue() == ControlModeValue.StaticBrake);
    SmartDashboard.putBoolean("try to climb", shouldClose() && RobotState.isEnabled());
    SmartDashboard.putNumber(
        "canrange distance", m_canRange.getDistance().getValueAsDouble() * 100.0);
  }
}
