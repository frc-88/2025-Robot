package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Armevator extends SubsystemBase {
  private final TalonFX m_elevatorMain =
      new TalonFX(Constants.ELEVATOR_MAIN_MOTOR, Constants.RIO_CANBUS);
  private final TalonFX m_elevatorFollower =
      new TalonFX(Constants.ELEVATOR_FOLLOWER_MOTOR, Constants.RIO_CANBUS);
  private final TalonFX m_arm = new TalonFX(Constants.ELEVATOR_ARM_MOTOR, Constants.RIO_CANBUS);
  private final CANcoder m_encoder = new CANcoder(Constants.ELEVATOR_ENCODER, Constants.RIO_CANBUS);

  private final PIDPreferenceConstants elevatorPID =
      new PIDPreferenceConstants("Armevator/Elevator/PID", 8, 0, 0, 0.15, 0, 0, 0, 0);
  private final DoublePreferenceConstant p_elevatorMaxVelocity =
      new DoublePreferenceConstant("Armevator/Elevator/MotionMagicVelocity", 60.0);
  private final DoublePreferenceConstant p_elevatorMaxAcceleration =
      new DoublePreferenceConstant("Armevator/Elevator/MotionMagicAcceleration", 120.0);
  private final DoublePreferenceConstant p_elevatorJerk =
      new DoublePreferenceConstant("Armevator/Elevator/MotionMagicJerk", 1200.0);
  private final DoublePreferenceConstant p_elevatorTargetInches =
      new DoublePreferenceConstant("Armevator/Elevator/TargetPositionInches", 6.0);

  private final PIDPreferenceConstants armPID =
      new PIDPreferenceConstants("Armevator/Arm/PID", 1, 0, 0, 0.12, 0, 0, 0, 0);
  private final DoublePreferenceConstant p_armMaxVelocity =
      new DoublePreferenceConstant("Armevator/Arm/MotionMagicVelocity", 40.0);
  private final DoublePreferenceConstant p_armMaxAcceleration =
      new DoublePreferenceConstant("Armevator/Arm/MotionMagicAcceleration", 80.0);
  private final DoublePreferenceConstant p_armJerk =
      new DoublePreferenceConstant("Armevator/Arm/MotionMagicJerk", 0.0);
  private final DoublePreferenceConstant p_armTargetDegrees =
      new DoublePreferenceConstant("Armevator/Arm/TargetPositionDegrees", 0.0);
  private final DoublePreferenceConstant p_armTiltAngle =
      new DoublePreferenceConstant("Armevator/Arm/TiltAngle", 5.0);

  private final DoublePreferenceConstant p_armEncoderOffset =
      new DoublePreferenceConstant("Armevator/Arm/EncoderOffset", -0.154785);

  private final MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);

  private final Debouncer elevatorDebouncer = new Debouncer(1.0);
  private final DigitalInput m_magnetInput = new DigitalInput(7);
  public final Trigger m_shouldCalibrate =
      new Trigger(
          () ->
              !m_magnetInput.get()
                  && elevatorDebouncer.calculate(
                      Math.abs(m_elevatorMain.getVelocity().getValueAsDouble()) < 0.1));

  private boolean m_calibrated = false;

  public Armevator() {
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration elevatorCfg = new TalonFXConfiguration();
    TalonFXConfiguration armCfg = new TalonFXConfiguration();
    CANcoderConfiguration cancg = new CANcoderConfiguration();
    cancg.MagnetSensor.MagnetOffset = p_armEncoderOffset.getValue();
    cancg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    elevatorCfg.Slot0.kP = elevatorPID.getKP().getValue();
    elevatorCfg.Slot0.kI = elevatorPID.getKI().getValue();
    elevatorCfg.Slot0.kD = elevatorPID.getKD().getValue();
    elevatorCfg.Slot0.kV = elevatorPID.getKF().getValue();

    elevatorCfg.MotionMagic.MotionMagicCruiseVelocity = p_elevatorMaxVelocity.getValue();
    elevatorCfg.MotionMagic.MotionMagicAcceleration = p_elevatorMaxAcceleration.getValue();
    elevatorCfg.MotionMagic.MotionMagicJerk = p_elevatorJerk.getValue();

    armCfg.Slot0.kP = armPID.getKP().getValue();
    armCfg.Slot0.kI = armPID.getKI().getValue();
    armCfg.Slot0.kD = armPID.getKD().getValue();
    armCfg.Slot0.kV = armPID.getKF().getValue();

    armCfg.MotionMagic.MotionMagicCruiseVelocity = p_armMaxVelocity.getValue();
    armCfg.MotionMagic.MotionMagicAcceleration = p_armMaxAcceleration.getValue();
    armCfg.MotionMagic.MotionMagicJerk = p_armJerk.getValue();

    m_elevatorMain.getConfigurator().apply(elevatorCfg);
    m_elevatorFollower.getConfigurator().apply(elevatorCfg);
    m_elevatorFollower.setControl(new Follower(Constants.ELEVATOR_MAIN_MOTOR, false));
    m_elevatorMain.setNeutralMode(NeutralModeValue.Brake);
    m_elevatorFollower.setNeutralMode(NeutralModeValue.Brake);

    m_arm.getConfigurator().apply(armCfg);
    m_encoder.getConfigurator().apply(cancg);
    m_arm.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getArmAngle() {
    return m_arm.getPosition().getValueAsDouble() * Constants.ARM_ROTATIONS_TO_DEGREES;
  }

  public boolean onTarget(double position) {
    return Math.abs(getElevatorPositionInches() - position) < 0.2;
  }

  private void armCalibrate() {
    m_arm.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble() * 112.0);
  }

  private void elevatorCalibrate() {
    m_elevatorMain.setPosition(0.0);
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

  private void setL4Shoot() {
    elevatorSetPosition(Constants.ELEVATOR_L4_HEIGHT);
    if (getElevatorPositionInches() > (Constants.ELEVATOR_L4_HEIGHT - 2)) {
      armSetAngle(-40.0);
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

  public boolean isArmZero() {
    return Math.abs(getArmAngle()) < 1.2;
  }

  public boolean isArmOnPosition() {
    return Math.abs(getArmAngle()) < 1.2;
  }

  private void elevatorSetSlowSpeed() {
    m_elevatorMain.setControl(new DutyCycleOut(0.1));
  }

  private void armSetSlowSpeed() {
    m_arm.setControl(new DutyCycleOut(0.1));
  }

  private void elevatorSetCalibrateSpeed() {
    m_elevatorMain.setControl(new DutyCycleOut(-0.1));
  }

  private void stowArm() {
    armGoToZero();
  }

  private void stowElevator() {
    elevatorSetPosition(0.0);
  }

  private void goToOneInch() {
    elevatorSetPosition(1.0);
  }

  public void stowBoth() {
    stowArm();
    stowElevator();
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

  public Command elevatorCalibrateFactory() {
    return new InstantCommand(() -> elevatorCalibrate(), this);
  }

  public Command L2Algae() {
    return new RunCommand(
        () -> {
          armSetAngle(30.0);
          elevatorSetPosition(8.5);
        },
        this);
  }

  public Command L3Algae() {
    return new RunCommand(
        () -> {
          armSetAngle(30.0);
          elevatorSetPosition(17.0);
        },
        this);
  }

  public Command calibrateElevatorFactory() {
    return new ConditionalCommand(
        goToOneInchFactory()
            .until(() -> onTarget(1.0))
            .andThen(new RunCommand(() -> stowElevator())),
        new RunCommand(() -> stowElevator(), this),
        () -> getElevatorPositionInches() > 1.0);
  }

  public Command calibrateBothFactory() {
    return new InstantCommand(
        () -> {
          armCalibrate();
          elevatorCalibrate();
          m_calibrated = true;
        },
        this);
  }

  public Command algaePickupFactory() {
    return new RunCommand(
        () -> {
          armGotoAlgaePickup();
          stowElevator();
        },
        this);
  }

  public Command goToOneInchFactory() {
    return new RunCommand(() -> goToOneInch(), this);
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

  public Command shootInNetFactory() {
    return new RunCommand(() -> setL4Shoot(), this);
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
    SmartDashboard.putNumber("Arm Position", getArmAngle());
  }
}
