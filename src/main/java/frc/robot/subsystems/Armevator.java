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
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

// take two subsystems
// then squish their names together
// portmanteau robot

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
  private final DoublePreferenceConstant p_armAngleNet =
      new DoublePreferenceConstant("Armevator/Arm/NetAngle", -30.0);

  private final DoublePreferenceConstant p_currentLimit =
      new DoublePreferenceConstant("Armevator/Elevator/CurrentLimit", 40);

  private final DoublePreferenceConstant p_currentLimitSupply =
      new DoublePreferenceConstant("Armevator/Elevator/SupplyCurrentLimit", 40);
  private final MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);

  private final Debouncer elevatorDebouncer = new Debouncer(0.5);
  private final DigitalInput m_magnetInput = new DigitalInput(7);
  public final Trigger m_shouldCalibrate =
      new Trigger(
          () ->
              !m_magnetInput.get()
                  && elevatorDebouncer.calculate(
                      Math.abs(m_elevatorMain.getVelocity().getValueAsDouble()) < 0.1));

  private BooleanSupplier m_safeToMove;
  private boolean m_calibrated = false;

  public Armevator(BooleanSupplier safeToMove) {
    m_safeToMove = safeToMove;
    configureTalons();
    armCalibrate();
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

    elevatorCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorCfg.CurrentLimits.StatorCurrentLimit = p_currentLimit.getValue();

    elevatorCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorCfg.CurrentLimits.SupplyCurrentLimit = p_currentLimitSupply.getValue();

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

  public boolean isReady() {
    return m_elevatorMain.isConnected()
        && m_elevatorFollower.isConnected()
        && m_arm.isConnected()
        && m_encoder.isConnected()
        && isArmInStartingPosition()
        && isElevatorDown();
  }

  @AutoLogOutput(key = "Armevator/armAngle")
  public double getArmAngle() {
    return m_arm.getPosition().getValueAsDouble() * Constants.ARM_ROTATIONS_TO_DEGREES;
  }

  public boolean onTarget(double position) {
    return Math.abs(getElevatorPositionInches() - position) < 1.0;
  }

  private void armCalibrate() {
    m_arm.setPosition(m_encoder.getAbsolutePosition().getValueAsDouble() * 48.0);
  }

  private void elevatorCalibrate() {
    m_elevatorMain.setPosition(0.0);
  }

  @AutoLogOutput(key = "Armevator/elevatorPosition")
  public double getElevatorPositionInches() {
    return m_elevatorMain.getPosition().getValueAsDouble() * Constants.ELEVATOR_ROTATIONS_TO_INCHES;
  }

  private void elevatorSetPosition(double position) {
    if (m_safeToMove.getAsBoolean()) {
      m_elevatorMain.setControl(
          motionmagicrequest
              .withPosition(position / Constants.ELEVATOR_ROTATIONS_TO_INCHES)
              .withFeedForward(0.056));
    } else {
      m_elevatorMain.setControl(
          motionmagicrequest
              .withPosition(m_elevatorMain.getPosition().getValueAsDouble())
              .withFeedForward(0.056));
    }
  }

  private void elevatorSetPositionSlow(double position) {
    if (m_safeToMove.getAsBoolean()) {
      m_elevatorMain.setControl(
          motionmagicrequest
              .withPosition(position / Constants.ELEVATOR_ROTATIONS_TO_INCHES)
              .withFeedForward(-3.0));
    } else {
      m_elevatorMain.setControl(
          motionmagicrequest
              .withPosition(m_elevatorMain.getPosition().getValueAsDouble())
              .withFeedForward(0.056));
    }
  }

  private void armSetAngle(double angle) {
    m_arm.setControl(motionmagicrequest.withPosition(angle / Constants.ARM_ROTATIONS_TO_DEGREES));
  }

  private void armSetAngleSlow(double angle) {
    m_arm.setControl(
        motionmagicrequest
            .withPosition(angle / Constants.ARM_ROTATIONS_TO_DEGREES)
            .withFeedForward(-0.25));
  }

  private void armGoToTiltAngle() {
    armSetAngle(p_armTiltAngle.getValue());
  }

  private void armGotoAlgaePickup() {
    armSetAngle(35.0);
  }

  private void armGotoPrefPosition() {
    armSetAngle(p_armTargetDegrees.getValue());
  }

  private void armGoToZero() {
    armSetAngle(0.0);
  }

  private void armGoToAlgaeStow() {
    armSetAngle(Constants.ALGAE_STOW_ANGLE);
  }

  private void setL4() {
    elevatorSetPosition(Constants.ELEVATOR_L4_HEIGHT);
    if (getElevatorPositionInches() > (Constants.ELEVATOR_L4_HEIGHT - 10.0)) {
      armSetAngle(Constants.ARM_L4_ANGLE);
    } else {
      armSetAngle(Constants.ARM_L4_SAFE_ANGLE);
    }
  }

  private void setL4Shoot() {
    elevatorSetPosition(Constants.ELEVATOR_L4_HEIGHT);
    if (getElevatorPositionInches() > (Constants.ELEVATOR_L4_HEIGHT - 2)) {
      armSetAngle(0);
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

  public boolean atL2() {
    return Math.abs(getElevatorPositionInches() - Constants.ELEVATOR_L2_HEIGHT) < 0.2;
  }

  public boolean atL3() {
    return Math.abs(getElevatorPositionInches() - Constants.ELEVATOR_L3_HEIGHT) < 0.2;
  }

  public boolean atL4() {
    return Math.abs(getElevatorPositionInches() - Constants.ELEVATOR_L4_HEIGHT) < 0.2
        && Math.abs(getArmAngle() - Constants.ARM_L4_ANGLE) < 1.0;
  }

  public boolean atShootHeight() {
    return getElevatorPositionInches() > 20.0;
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

  public boolean isArmInStartingPosition() {
    return Math.abs(getArmAngle()) < 5.0;
  }

  public boolean isArmOnPosition() {
    return Math.abs(getArmAngle()) < 30.0;
  }

  public boolean isArmOnAlgaePosition() {
    return Math.abs(getArmAngle() - Constants.ALGAE_STOW_ANGLE) < 1.0;
  }

  public boolean isElevatorDown() {
    return !m_magnetInput.get();
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

  private void stowArmAlgae() {
    armGoToAlgaeStow();
  }

  private void stowArmAlgaeProcessor() {
    armSetAngleSlow(30.0);
  }

  private void stowArmAlgaeProcessor(double position) {
    armSetAngleSlow(position);
  }

  private void stowElevator() {
    elevatorSetPosition(0.0);
  }

  private void stowElevatorAlgae() {
    elevatorSetPosition(11.0);
  }

  private void stowElevatorAlgaeProcessor() {
    elevatorSetPositionSlow(0.0);
  }

  private void stowElevatorAlgaeProcessor(double position) {
    elevatorSetPositionSlow(position);
  }

  private void goToOneInch() {
    elevatorSetPosition(1.0);
  }

  public void stowBoth() {
    stowArm();
    stowElevator();
  }

  public boolean elevatorAtProcessorPosition() {
    return Math.abs(getElevatorPositionInches() - 3.0) < 0.4;
  }

  public boolean elevatorAboveDoghouse() {
    return getElevatorPositionInches() > 24;
  }

  public boolean elevatorAtZero() {
    return Math.abs(getElevatorPositionInches()) < 0.25;
  }

  public boolean atMode(IntSupplier i) {
    if (i.getAsInt() == 4) {
      return getArmAngle() > (Constants.ARM_L4_ANGLE - 1.2)
          && getArmAngle() < (Constants.ARM_L4_ANGLE + 2.9);
    } else if (i.getAsInt() == 3) {
      return Math.abs(getElevatorPositionInches() - Constants.ELEVATOR_L3_HEIGHT) < 1.0;
    } else {
      return Math.abs(getElevatorPositionInches() - Constants.ELEVATOR_L2_HEIGHT) < 1.0;
    }
  }

  public void setAlgaeElevatorPosition(IntSupplier sector) {
    if (sector.getAsInt() == 1 || sector.getAsInt() == 3 || sector.getAsInt() == 5) {
      elevatorSetPosition(14.5);
    } else {
      elevatorSetPosition(7.0);
    }
  }

  public boolean elevatorAtAlgaePositon(IntSupplier sector) {
    if (sector.getAsInt() == 1 || sector.getAsInt() == 3 || sector.getAsInt() == 5) {
      return Math.abs(getElevatorPositionInches() - 14.5) < 0.2;
    } else {
      return Math.abs(getElevatorPositionInches() - 7.0) < 0.2;
    }
  }

  public void armGotoAlgaePickup(IntSupplier sector) {
    armSetAngle(30.0);
  }

  public Command stowArmFactory() {
    return new RunCommand(() -> stowArm(), this);
  }

  public Command stowArmAlgaeFactory() {
    return new RunCommand(
        () -> {
          stowArmAlgae();
          stowElevatorAlgae();
        },
        this);
  }

  public Command stowProcessor() {
    return new RunCommand(
            () -> {
              if (getArmAngle() > 22.0) {
                stowElevatorAlgaeProcessor(3.0);
              }
              stowArmAlgaeProcessor();
            },
            this)
        .until(() -> elevatorAtProcessorPosition())
        .andThen(
            new RunCommand(
                    () -> {
                      stowElevatorAlgaeProcessor();
                      stowArmAlgaeProcessor();
                    })
                .until(() -> elevatorAtZero())
                .andThen(
                    new RunCommand(
                        () -> {
                          stowElevatorAlgaeProcessor();
                          stowArmAlgaeProcessor(55.0);
                        },
                        this)));
  }

  public Command stowBothFactory() {
    return new RunCommand(() -> stowArmFactory(), this);
  }

  public Command calibrateArmFactory() {
    return new InstantCommand(() -> armCalibrate(), this);
  }

  public Command elevatorCalibrateFactory() {
    return new InstantCommand(() -> elevatorCalibrate());
  }

  public Command L2Algae() {
    return new RunCommand(
        () -> {
          armSetAngle(30.0);
          elevatorSetPosition(7.0);
        },
        this);
  }

  public Command L3Algae() {
    return new RunCommand(
        () -> {
          armSetAngle(30.0);
          elevatorSetPosition(14.5);
        },
        this);
  }

  public Command calibrateElevatorFactory() {
    return new ConditionalCommand(
        goToOneInchFactory()
            .until(() -> onTarget(1.0))
            .andThen(
                new RunCommand(
                    () -> {
                      stowElevator();
                      armGoToZero();
                    })),
        new RunCommand(
            () -> {
              stowElevator();
              armGoToZero();
            },
            this),
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
    return new RunCommand(
        () -> {
          goToOneInch();
          armGoToZero();
        },
        this);
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

  public Command AlgaestowFactory() {
    return stowArmAlgaeFactory();
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

  public Command defaultCommand(BooleanSupplier isAlgae) {
    return new ConditionalCommand(
        new ConditionalCommand(AlgaestowFactory(), stowFactory(), isAlgae),
        calibrateBothFactory(),
        () -> m_calibrated);
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

  public Command scoreAll(IntSupplier mode) {
    return new RunCommand(
        () -> {
          if (mode.getAsInt() == 2) {
            setL2();
          } else if (mode.getAsInt() == 3) {
            setL3();
          } else if (mode.getAsInt() == 4) {
            setL4();
          } else {
            armGoToZero();
            stowElevator();
          }
        },
        this);
  }

  public Command algae(IntSupplier sector) {
    return new RunCommand(
        () -> {
          if (sector.getAsInt() == 1 || sector.getAsInt() == 3 || sector.getAsInt() == 5) {
            armSetAngle(30.0);
            elevatorSetPosition(15.0);
          } else {
            armSetAngle(30.0);
            elevatorSetPosition(8.0);
          }
        },
        this);
  }

  public Command stowThenalgae(IntSupplier sector) {
    return new SequentialCommandGroup(
        new RunCommand(() -> armGoToZero(), this).until(() -> isArmOnPosition()),
        new RunCommand(
                () -> {
                  armGoToZero();
                  setAlgaeElevatorPosition(sector);
                },
                this)
            .until(() -> elevatorAtAlgaePositon(sector)),
        new RunCommand(
            () -> {
              armGotoAlgaePickup();
              setAlgaeElevatorPosition(sector);
            },
            this));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Positon", getElevatorPositionInches());
    SmartDashboard.putNumber("Arm Position", getArmAngle());
  }
}
