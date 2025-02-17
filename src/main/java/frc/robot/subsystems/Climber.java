// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Climber extends SubsystemBase {
  private DoublePreferenceConstant p_grippermaxVelocity =
      new DoublePreferenceConstant("Climber/gripperMotionMagicVelocity", 100);
  private DoublePreferenceConstant p_grippermaxAcceleration =
      new DoublePreferenceConstant("Climber/gripperMotionMagicAcceleration", 1000);
  private DoublePreferenceConstant p_grippermaxJerk =
      new DoublePreferenceConstant("Climber/gripperMotionMagicJerk", 100000);
  private PIDPreferenceConstants p_gripperPidPreferenceConstants =
      new PIDPreferenceConstants("Climber/gripperPID", 0.0, 0.0, 0.0, 0.12, 0.0, 0.0, 0.0, 0.0);

  private DoublePreferenceConstant p_pivotmaxVelocity =
      new DoublePreferenceConstant("Climber/pivotMotionMagicVelocity", 100);
  private DoublePreferenceConstant p_pivotmaxAcceleration =
      new DoublePreferenceConstant("Climber/pivotMotionMagicAcceleration", 1000);
  private DoublePreferenceConstant p_pivotmaxJerk =
      new DoublePreferenceConstant("Climber/pivotMotionMagicJerk", 100000);
  private PIDPreferenceConstants p_pivotPidPreferenceConstants =
      new PIDPreferenceConstants("Climber/pivotPID", 0.0, 0.0, 0.0, 0.12, 0.0, 0.0, 0.0, 0.0);

  private PIDPreferenceConstants p_gasmotorPID =
      new PIDPreferenceConstants("Climber/GasMotorPID", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  private DoublePreferenceConstant p_pivotLimit =
      new DoublePreferenceConstant("Climber/PivotLimit", 100);
  private DoublePreferenceConstant p_gripperLimit =
      new DoublePreferenceConstant("Climber/GripperLimit", 100);
  private DoublePreferenceConstant p_gripperStowSpeed =
      new DoublePreferenceConstant("Climber/GripperStowSpeed", 0.01);
  private DoublePreferenceConstant p_pivotStowSpeed =
      new DoublePreferenceConstant("Climber/PivotStowSpeed", 0.01);
  private DoublePreferenceConstant p_gasmotorLimit =
      new DoublePreferenceConstant("Climber/GasMotorLimit", 10.0);

  private DoublePreferenceConstant p_gripperClosedTorque =
      new DoublePreferenceConstant("Climber/gripperClosedTorque", 0.0);
  private DoublePreferenceConstant p_gripperStallTorque =
      new DoublePreferenceConstant("Climber/gripperStallTorque", 0.0);
  private DoublePreferenceConstant p_pivotTorque =
      new DoublePreferenceConstant("Climber/PivotTorque", 0.0);
  private DoublePreferenceConstant p_gasmotorSpeed =
      new DoublePreferenceConstant("Climber/Gasmotorspeed", 0.2);
  private DoublePreferenceConstant p_gasmotorPositionInches =
      new DoublePreferenceConstant("Climber/GasMotorPositionInches", 0.2);
  private DoublePreferenceConstant p_gasmotorPositionRotations =
      new DoublePreferenceConstant("Climber/GasMotorPositionRotations", 0.2);
  private DoublePreferenceConstant p_gripperPosition =
      new DoublePreferenceConstant("Climber/GripperMotorAngle", 0.0);

  // private final TalonFX m_pivot = new TalonFX(Constants.CLIMBER_PIVOT_MOTOR,
  // Constants.RIO_CANBUS);
  private final TalonFX m_gripper =
      new TalonFX(Constants.CLIMBER_GRIPPER_MOTOR, Constants.RIO_CANBUS);
  private final TalonFX m_gasmotor = new TalonFX(Constants.CLIMBER_GAS_MOTOR, Constants.RIO_CANBUS);
  private final CANcoder m_climberEncoder =
      new CANcoder(Constants.CLIMBER_ENCODER, Constants.RIO_CANBUS);
  private final CANrange m_canRange =
      new CANrange(Constants.CLIMBER_CANRANGE, Constants.RIO_CANBUS);

  private DigitalInput input = new DigitalInput(9);

  private boolean isCalibrated = false;
  // private double kPivotMotorRotationsToClimberPosition =
  // Constants.PIVOT_MOTOR_ROTATIONS_TO_CLIMBER_POSITION;
  private double kGripperMotorRotationsToPosition = Constants.GRIPPER_MOTOR_ROTATIONS_TO_POSITION;

  // private final DutyCycleOut m_pivotRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut m_gripperRequest = new DutyCycleOut(0.0);
  private MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0);
  // private TorqueCurrentFOC gripperClosedtorque =
  // new TorqueCurrentFOC(p_gripperClosedTorque.getValue()).withMaxAbsDutyCycle(0.75);
  // private TorqueCurrentFOC gripperStalltorque =
  // new TorqueCurrentFOC(p_gripperStallTorque.getValue());
  // private TorqueCurrentFOC pivottorque = new
  // TorqueCurrentFOC(p_pivotTorque.getValue()).withMaxAbsDutyCycle(0.25);
  private PositionDutyCycle position = new PositionDutyCycle(0.0);
  private boolean isClimbing = false;

  private Debouncer climberDebouncer = new Debouncer(1.0);
  public Trigger onDisable = new Trigger(() -> shouldEnableBrake());
  public Trigger shouldClose = new Trigger(() -> shouldClose() && isClimbing);

  /** Creates a new Climber. */
  public Climber() {
    configureTalons();
    m_gripper.setPosition(0.0);
    // m_pivot.setPosition(0.0);
  }

  private void configureTalons() {
    TalonFXConfiguration pivotcfg = new TalonFXConfiguration();
    TalonFXConfiguration grippercfg = new TalonFXConfiguration();
    TalonFXConfiguration gasmotorcfg = new TalonFXConfiguration();
    // CANcoderConfiguration cfg = new CANcoderConfiguration();
    CANrangeConfiguration canRangecfg = new CANrangeConfiguration();

    MotionMagicConfigs pivot_mm = pivotcfg.MotionMagic;
    MotionMagicConfigs gripper_mm = grippercfg.MotionMagic;

    gripper_mm.MotionMagicCruiseVelocity = p_grippermaxVelocity.getValue();
    gripper_mm.MotionMagicAcceleration = p_grippermaxAcceleration.getValue();
    gripper_mm.MotionMagicJerk = p_grippermaxJerk.getValue();

    pivot_mm.MotionMagicCruiseVelocity = p_pivotmaxVelocity.getValue();
    pivot_mm.MotionMagicAcceleration = p_pivotmaxAcceleration.getValue();
    pivot_mm.MotionMagicJerk = p_pivotmaxJerk.getValue();

    Slot0Configs pivotslot0 = pivotcfg.Slot0;
    Slot0Configs gripperslot0 = grippercfg.Slot0;
    Slot0Configs gasmotorslot0 = gasmotorcfg.Slot0;

    gripperslot0.kP = p_gripperPidPreferenceConstants.getKP().getValue();
    gripperslot0.kI = p_gripperPidPreferenceConstants.getKI().getValue();
    gripperslot0.kD = p_gripperPidPreferenceConstants.getKD().getValue();
    gripperslot0.kV = p_gripperPidPreferenceConstants.getKF().getValue();
    gripperslot0.kS = p_gripperPidPreferenceConstants.getKS().getValue();

    pivotslot0.kP = p_pivotPidPreferenceConstants.getKP().getValue();
    pivotslot0.kI = p_pivotPidPreferenceConstants.getKI().getValue();
    pivotslot0.kD = p_pivotPidPreferenceConstants.getKD().getValue();
    pivotslot0.kV = p_pivotPidPreferenceConstants.getKF().getValue();
    pivotslot0.kS =
        p_pivotPidPreferenceConstants
            .getKS()
            .getValue(); // Approximately 0.25V to get the mechanism moving

    gasmotorslot0.kP = p_gasmotorPID.getKP().getValue();
    gasmotorslot0.kI = p_gasmotorPID.getKI().getValue();
    gasmotorslot0.kD = p_gasmotorPID.getKD().getValue();
    gasmotorslot0.kV = p_gasmotorPID.getKF().getValue();
    gasmotorslot0.kS = p_gasmotorPID.getKS().getValue();

    SoftwareLimitSwitchConfigs softLimits = pivotcfg.SoftwareLimitSwitch;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = p_pivotLimit.getValue();

    // VoltageConfigs grippervoltage = grippercfg.Voltage;
    // grippervoltage.PeakForwardVoltage = 9.0;

    SoftwareLimitSwitchConfigs gripperSoftLimits = grippercfg.SoftwareLimitSwitch;
    gripperSoftLimits.ForwardSoftLimitEnable = false;
    gripperSoftLimits.ForwardSoftLimitThreshold = p_gripperLimit.getValue();

    SoftwareLimitSwitchConfigs gasmotorsoftlimtis = gasmotorcfg.SoftwareLimitSwitch;
    gasmotorsoftlimtis.ForwardSoftLimitEnable = true;
    gasmotorsoftlimtis.ForwardSoftLimitThreshold =
        (p_gasmotorLimit.getValue() / Constants.GAS_MOTOR_ROTATIONS_TO_LENGTH);
    gasmotorcfg.CurrentLimits.StatorCurrentLimitEnable = false;
    gasmotorcfg.CurrentLimits.SupplyCurrentLimitEnable = false;
    // gasmotorcfg.CurrentLimits.SupplyCurrentLimit = 50.0;
    // m_pivot.getConfigurator().apply(pivotcfg);
    grippercfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_gripper.getConfigurator().apply(grippercfg);

    m_gasmotor.getConfigurator().apply(gasmotorcfg);

    m_gasmotor.setNeutralMode(NeutralModeValue.Brake);

    m_gripper.setNeutralMode(NeutralModeValue.Brake);
  }

  public Trigger shouldGripperClose() {
    return shouldClose;
  }

  public boolean shouldClose() {
    return !input.get()
        && Units.metersToInches(m_canRange.getDistance().getValueAsDouble()) > 8.5
        && Units.metersToInches(m_canRange.getDistance().getValueAsDouble()) < 9.5;
  }

  public boolean onTarget() {
    return Math.abs(
            getPositionGasMotor()
                - ((p_gasmotorPositionInches.getValue() / Constants.GAS_MOTOR_ROTATIONS_TO_LENGTH)
                    * 1.21))
        < 1.0;
  }

  public boolean shouldEnableBrake() {
    return RobotState.isDisabled() && getPositionGasMotor() < 40.0;
  }

  public void gasMotorBrakeMode() {
    m_gasmotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void gasMotorNeutralMode() {
    m_gasmotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void pivotNeutralGrabberOpen() {
    // m_pivot.setNeutralMode(NeutralModeValue.Coast);
    m_gripper.setControl(
        m_motionMagic.withPosition(
            p_gripperPosition.getValue() / Constants.GRIPPER_MOTOR_ROTATIONS_TO_POSITION));
  }

  public void pivotNeutralGrabberClosed() {
    // m_pivot.setNeutralMode(NeutralModeValue.Coast);
    m_gripper.setControl(m_motionMagic.withPosition(0.0));
  }

  public void pivotUpGrabberClosed() {
    // m_pivot.setControl(pivottorque);
    m_gripper.setControl(m_motionMagic.withPosition(0.0));
  }

  public void stow() {
    if (isCalibrated) {
      m_gripper.setControl(m_motionMagic.withPosition(0.0));
      // m_pivot.setControl(m_motionMagic.withPosition(0.0));
    } else {
      m_gripper.setControl(new DutyCycleOut(-p_gripperStowSpeed.getValue()));
      // m_pivot.setControl(new DutyCycleOut(-p_pivotStowSpeed.getValue()));

      if (climberDebouncer.calculate(m_gripper.getVelocity().getValueAsDouble() > -1)
      /*climberDebouncer.calculate(m_pivot.getVelocity().getValueAsDouble() > -1)*/ ) {
        calibrate();
        isCalibrated = true;
      }
    }
  }

  public double getPositionGasMotor() {
    return m_gasmotor.getPosition().getValueAsDouble();
  }

  public void setGasMotorPostionInches() {
    isClimbing = true;
    m_gasmotor.setControl(
        position.withPosition(
            (p_gasmotorPositionInches.getValue() / Constants.GAS_MOTOR_ROTATIONS_TO_LENGTH)
                * 1.21));
  }

  public void setGasMotorPositionRotations() {
    m_gasmotor.setControl(position.withPosition(p_gasmotorPositionRotations.getValue()));
  }

  public void holdPostion() {
    m_gasmotor.setControl(
        new PositionDutyCycle(
            (p_gasmotorPositionInches.getValue() / Constants.GAS_MOTOR_ROTATIONS_TO_LENGTH)
                * 1.21));
    // m_pivot.setControl(new DutyCycleOut(0.0));
  }

  public void setGripperPosition(double position) {
    m_gripper.setControl(m_motionMagic.withPosition(position / kGripperMotorRotationsToPosition));
  }

  public void setPivotPosition(double position) {
    // m_pivot.setControl(m_motionMagic.withPosition(position /
    // kPivotMotorRotationsToClimberPosition));
  }

  public void setGasMotorSpeed() {
    m_gasmotor.setControl(new DutyCycleOut(p_gasmotorSpeed.getValue()));
  }

  public void stopGasMotor() {
    m_gasmotor.setControl(new DutyCycleOut(0.0));
  }

  public void calibrateGasMotor() {
    m_gasmotor.setPosition(0.0);
  }

  public void calibrate() {
    m_gripper.setPosition(0.0);
    // m_pivot.setPosition(0.0);
    m_gasmotor.setPosition(0.0);
  }

  public Trigger shouldBrake() {
    return onDisable;
  }

  public void calibrateEncoder() {
    m_climberEncoder.setPosition(0.0);
  }

  public double getAngleOfClimber() {
    return m_climberEncoder.getPosition().getValueAsDouble()
        * Constants.CLIMBER_ENCODER_ROTATIONS_TO_ANGLE;
  }

  public double getVelocity() {
    return m_gasmotor.getVelocity().getValueAsDouble();
  }

  public Command stowFactory() {
    return new RunCommand(() -> stow(), this)
        .beforeStarting(() -> climberDebouncer.calculate(false));
  }

  public Command holdPositionFactory() {
    return new RunCommand(() -> holdPostion(), this);
  }

  public Command pivotNeutralGrabberOpenFactory() {
    return new RunCommand(
        () -> {
          pivotNeutralGrabberOpen();
          setGasMotorPostionInches();
        },
        this);
  }

  public Command pivotNeutralGrabberClosedFactory() {
    return new RunCommand(() -> pivotNeutralGrabberClosed(), this);
  }

  public Command pivotUpGrabberClosedFactory() {
    return new RunCommand(() -> pivotUpGrabberClosed(), this);
  }

  public Command runGasMotorInchesFactory() {
    return new RunCommand(() -> setGasMotorPostionInches(), this);
  }

  public Command runGasMotorRotationsFactory() {
    return new RunCommand(() -> setGasMotorPositionRotations(), this);
  }

  public Command stopGasMotorFactory() {
    return new RunCommand(() -> stopGasMotor(), this);
  }

  public Command gasMotorBrakeModeFactory() {
    return new InstantCommand(() -> gasMotorBrakeMode(), this);
  }

  public Command calibrateGasMotorFactory() {
    return new InstantCommand(() -> calibrateGasMotor(), this);
  }

  public Command setPositionFactory() {
    return new RunCommand(() -> setGasMotorPostionInches(), this);
  }

  public Command calibrateEncoderFactory() {
    return new InstantCommand(() -> calibrateEncoder(), this);
  }

  public Command setNeutralModeFactory() {
    return new InstantCommand(() -> gasMotorNeutralMode(), this);
  }

  public Command prepClimber() {
    return new RunCommand(
        () -> {
          setGasMotorPostionInches();
          pivotNeutralGrabberOpen();
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler runn s
    SmartDashboard.putNumber("gas motor position", getPositionGasMotor());
    SmartDashboard.putNumber(
        "gas motor desired position",
        p_gasmotorPositionInches.getValue() / Constants.GAS_MOTOR_ROTATIONS_TO_LENGTH);
    SmartDashboard.putNumber("Encoder position", m_climberEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Angle", getAngleOfClimber());
    SmartDashboard.putNumber(
        "Climber CAN Range Distance",
        Units.metersToInches(m_canRange.getDistance().getValueAsDouble()));
    SmartDashboard.putBoolean("Sensor ouput", input.get());
  }
}
