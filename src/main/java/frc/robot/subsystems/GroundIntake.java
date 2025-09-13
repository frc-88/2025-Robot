package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class GroundIntake extends SubsystemBase {
  private final CANcoder m_intakeCANcoder = new CANcoder(7, Constants.RIO_CANBUS);
  private final TalonFX m_pivot_motor =
      new TalonFX(Constants.INTAKE_PIVOT_MOTOR, Constants.RIO_CANBUS);
  private final TalonFX m_roller_motor =
      new TalonFX(Constants.INTAKE_ROLLER_MOTOR, Constants.RIO_CANBUS);
  // private final DoublePreferenceConstant p_intakeSpeed = new
  // DoublePreferenceConstant("GroundIntake/PivotMotor/IntakeSpeed", );
  private final DoublePreferenceConstant p_pivotMotorCurrentLimit =
      new DoublePreferenceConstant("GroundIntake/PivotMotor/CurrentLimit", 60);
  private final DoublePreferenceConstant p_intakeEncoderOffset =
      new DoublePreferenceConstant("GroundIntake/PivotMotor/MagnetOffset", -0.402100);
  private final DoublePreferenceConstant p_rollerMotorCurrentLimit =
      new DoublePreferenceConstant("GroundIntake/RollerMotor/CurrentLimit", 60);
  private final MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);
  private final PIDPreferenceConstants intakePID =
      new PIDPreferenceConstants("GroundIntake/PivotMotor/PID", 0.5, 0, 0, 0.12, 0, 0, 0, 0);
  private final PIDPreferenceConstants rollerPID =
      new PIDPreferenceConstants("GroundIntake/RollerMotor/PID", 0.5, 0, 0, 0.12, 0, 0, 0, 0);
  private final DoublePreferenceConstant p_intakeMaxVelocity =
      new DoublePreferenceConstant("GroundIntake/PivotMotor/MotionMagicVelocity", 40.0);
  private final DoublePreferenceConstant p_intakeMaxAcceleration =
      new DoublePreferenceConstant("GroundIntake/PivotMotor/MotionMagicAcceleration", 80.0);
  private final DoublePreferenceConstant p_intakeJerk =
      new DoublePreferenceConstant("GroundIntake/PivotMotor/MotionMagicJerk", 0.0);
  private final DoublePreferenceConstant p_rollerSpeed =
      new DoublePreferenceConstant("GroundIntake/RollerMotor/Speed", 0.0);
  private final DutyCycleOut m_rollerRequest = new DutyCycleOut(0.0);
  private final Debouncer intakeDebouncer = new Debouncer(0.5);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0.0);

  public GroundIntake() {
    CANcoderConfiguration intakeCANcoderConfiguration = new CANcoderConfiguration();
    intakeCANcoderConfiguration.MagnetSensor.MagnetOffset = p_intakeEncoderOffset.getValue();
    intakeCANcoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.Clockwise_Positive;

    m_intakeCANcoder.getConfigurator().apply(intakeCANcoderConfiguration);

    TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration();
    pivotMotorConfiguration.CurrentLimits.SupplyCurrentLimit = p_pivotMotorCurrentLimit.getValue();
    pivotMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotMotorConfiguration.Slot0.kP = intakePID.getKP().getValue();
    pivotMotorConfiguration.Slot0.kI = intakePID.getKI().getValue();
    pivotMotorConfiguration.Slot0.kD = intakePID.getKD().getValue();
    pivotMotorConfiguration.Slot0.kV = intakePID.getKF().getValue();

    pivotMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = p_intakeMaxVelocity.getValue();
    pivotMotorConfiguration.MotionMagic.MotionMagicAcceleration =
        p_intakeMaxAcceleration.getValue();
    pivotMotorConfiguration.MotionMagic.MotionMagicJerk = p_intakeJerk.getValue();

    pivotMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_pivot_motor.getConfigurator().apply(pivotMotorConfiguration);

    // roller: negative is in; positive is out
    TalonFXConfiguration rollerMotorConfiguration = new TalonFXConfiguration();
    rollerMotorConfiguration.CurrentLimits.SupplyCurrentLimit =
        p_rollerMotorCurrentLimit.getValue();
    rollerMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotorConfiguration.Slot0.kP = rollerPID.getKP().getValue();
    rollerMotorConfiguration.Slot0.kI = rollerPID.getKI().getValue();
    rollerMotorConfiguration.Slot0.kD = rollerPID.getKD().getValue();

    m_roller_motor.getConfigurator().apply(rollerMotorConfiguration);

    m_roller_motor.setNeutralMode(NeutralModeValue.Brake);
    m_pivot_motor.setNeutralMode(NeutralModeValue.Brake);

    intakeCalibrate();
  }

  private void intakeSetAngle(double angle) {
    m_pivot_motor.setControl(
        motionmagicrequest.withPosition(angle / Constants.INTAKE_ROTATIONS_TO_DEGREES));

  }

  private void rollerSetSpeed(double speed) {
    m_roller_motor.setControl(
        m_velocityRequest.withVelocity(speed));
  }      

  private void intakeGoToGround() {
    intakeSetAngle(115);
  }

  private void intakeGoToScore() {
    intakeSetAngle(20.0);
  }

  private void intakeGoToStow() {
    intakeSetAngle(0);
  }

  private void intakeRollerInFast() {
    setRollerSpeed(-.5);
  }

  private void intakeRollerOut() {
    setRollerSpeed(.3);
  }

  private void intakeRollerStop() {
    setRollerSpeed(0);
  }

  private void setRollerSpeed(double output, boolean slowRamp) {
    OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
    config.DutyCycleOpenLoopRampPeriod = slowRamp ? .2 : 0;
    m_roller_motor.getConfigurator().apply(config);
    m_roller_motor.setControl(m_rollerRequest.withOutput(output));
  }

  private void setRollerSpeed(double output) {
    setRollerSpeed(output, false);
  }

  private void intakeCalibrate() {
    m_pivot_motor.setPosition(
        m_intakeCANcoder.getAbsolutePosition().getValueAsDouble() * 131.25); // 78.75
  }

  public Command stowFactory() {
    return new RunCommand(
        () -> {
          intakeGoToStow();
          intakeRollerStop();
        },
        this);
  }

  public Command groundIntakeFactory() {
    return new RunCommand(
            () -> {
              intakeGoToGround();
              intakeRollerInFast();
            },
            this)
        .until(
            () ->
                intakeDebouncer.calculate(
                    m_roller_motor.getSupplyCurrent().getValueAsDouble() > 25.0));
  }

  public Command goToScoreFactory() {
    return new RunCommand(
        () -> {
          intakeGoToScore();
          intakeRollerStop();
        },
        this);
  }

  public Command intakeShoot() {
    return new RunCommand(
            () -> {
              rollerSetSpeed(p_rollerSpeed.getValue());
            },
            this)
        .withTimeout(0.7);
  }
}
