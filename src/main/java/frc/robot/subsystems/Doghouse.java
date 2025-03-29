// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Doghouse extends SubsystemBase {
  private final TalonFX m_funnel = new TalonFX(Constants.DOGHOUSE_FUNNEL_MOTOR, "rio");
  private final TalonFX m_manipulator =
      new TalonFX(Constants.ELEVATOR_MANIPULATOR_MOTOR, Constants.RIO_CANBUS);
  private final CANrange m_doghousCANRange =
      new CANrange(Constants.DOGHOUSE_CANRANGE, Constants.RIO_CANBUS);
  private final CANrange m_coralRange =
      new CANrange(Constants.CORAL_CANRANGE, Constants.RIO_CANBUS);
  private final CANrange m_reefRange = new CANrange(Constants.REEF_CANRANGE, Constants.RIO_CANBUS);

  private final DoublePreferenceConstant p_funnelSpeed =
      new DoublePreferenceConstant("Doghouse/Funnel/Speed", 1);
  private final DoublePreferenceConstant p_funnelCurrentLimit =
      new DoublePreferenceConstant("Doghouse/Funnel/CurrentLimit", 20);
  private final DoublePreferenceConstant p_manipulatorInSpeed =
      new DoublePreferenceConstant("Doghouse/Manipulator/InSpeed", -0.2);
  private final DoublePreferenceConstant p_manipulatorShootSpeed =
      new DoublePreferenceConstant("Doghouse/Manipulator/ShootSpeed", -0.3);
  private final DoublePreferenceConstant p_manipulatorCurrentLimit =
      new DoublePreferenceConstant("Doghouse/Manipulator/CurrentLimit", 120);
  private final PIDPreferenceConstants p_manipulatorPID =
      new PIDPreferenceConstants("Doghouse/Manipulator/PID");

  private final DutyCycleOut m_funnelRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut m_manipulatorRequest = new DutyCycleOut(0.0);
  private final TorqueCurrentFOC m_algaePickupRequest = new TorqueCurrentFOC(-80.0);

  private boolean algaeMode = false;

  private boolean m_coralCaptured = false;
  private boolean m_algaeCaptured = false;
  private Debouncer m_algaeDebouncer = new Debouncer(1.0);

  private PositionVoltage request = new PositionVoltage(0.0);
  // who made the doghouse?
  // and why is it named like that?
  // ask us in the pit!

  public Doghouse() {
    // configure funnel
    TalonFXConfiguration doghouseConfiguration = new TalonFXConfiguration();
    doghouseConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    doghouseConfiguration.CurrentLimits.SupplyCurrentLimit = p_funnelCurrentLimit.getValue();
    doghouseConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_funnel.getConfigurator().apply(doghouseConfiguration);

    // configure manipulator
    TalonFXConfiguration manipulatorConfiguration = new TalonFXConfiguration();
    manipulatorConfiguration.CurrentLimits.SupplyCurrentLimit =
        p_manipulatorCurrentLimit.getValue();
    manipulatorConfiguration.Slot0.kP = p_manipulatorPID.getKP().getValue();
    manipulatorConfiguration.Slot0.kI = p_manipulatorPID.getKI().getValue();
    manipulatorConfiguration.Slot0.kD = p_manipulatorPID.getKD().getValue();
    manipulatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    manipulatorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    manipulatorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;
    manipulatorConfiguration.HardwareLimitSwitch.ForwardLimitType =
        ForwardLimitTypeValue.NormallyOpen;
    manipulatorConfiguration.HardwareLimitSwitch.ForwardLimitSource =
        ForwardLimitSourceValue.RemoteCANrange;
    manipulatorConfiguration.HardwareLimitSwitch.ForwardLimitRemoteSensorID =
        Constants.DOGHOUSE_CANRANGE;
    m_manipulator.getConfigurator().apply(manipulatorConfiguration);
    m_manipulator.setNeutralMode(NeutralModeValue.Brake);
    configureCANrange();
    m_manipulator.setNeutralMode(NeutralModeValue.Brake);
  }

  private void configureCANrange() {
    CANrangeConfiguration coralRangecfg = new CANrangeConfiguration();
    CANrangeConfiguration doghouscfg = new CANrangeConfiguration();
    CANrangeConfiguration reefRangecfg = new CANrangeConfiguration();
    coralRangecfg.FovParams.FOVRangeX = 6.75;
    doghouscfg.FovParams.FOVRangeX = 7.5;
    coralRangecfg.FovParams.FOVRangeY = 6.75;
    doghouscfg.FovParams.FOVRangeY = 27.0;

    reefRangecfg.FovParams.FOVRangeX = 15.0;
    reefRangecfg.FovParams.FOVRangeY = 7.0;

    reefRangecfg.ProximityParams.ProximityThreshold = 0.28;
    reefRangecfg.ProximityParams.ProximityHysteresis = 0.03;
    reefRangecfg.ProximityParams.MinSignalStrengthForValidMeasurement = 1300;

    doghouscfg.ToFParams.UpdateFrequency = 50;
    coralRangecfg.ToFParams.UpdateFrequency = 50;

    doghouscfg.ProximityParams.ProximityThreshold = 0.15;
    doghouscfg.ProximityParams.ProximityHysteresis = 0.01;
    coralRangecfg.ProximityParams.ProximityThreshold = 0.5;

    coralRangecfg.ProximityParams.MinSignalStrengthForValidMeasurement = 40000;
    doghouscfg.ProximityParams.MinSignalStrengthForValidMeasurement = 13000;

    m_doghousCANRange.getConfigurator().apply(doghouscfg);
    m_coralRange.getConfigurator().apply(coralRangecfg);
    m_reefRange.getConfigurator().apply(reefRangecfg);
  }

  public boolean isReady() {
    return m_funnel.isConnected()
        && m_manipulator.isConnected()
        && m_doghousCANRange.isConnected()
        && m_coralRange.isConnected()
        && m_reefRange.isConnected()
        && hasCoral()
        && !isBlocked();
  }

  @AutoLogOutput(key = "DogHouse/reefDetected")
  public boolean getIsReefDetected() {
    return m_reefRange.getIsDetected().getValue();
  }

  public boolean hasCoralDebounced() {
    return m_algaeDebouncer.calculate(hasCoral());
  }

  private void setFunnelSpeed(double output) {
    m_funnel.setControl(m_funnelRequest.withOutput(output));
  }

  private void setManipulatorSpeed(double output) {
    m_manipulator.setControl(m_manipulatorRequest.withOutput(output));
  }

  @AutoLogOutput(key = "DogHouse/hasCoral")
  public boolean hasCoral() {
    return m_coralRange.getIsDetected().getValue();
  }

  @AutoLogOutput(key = "DogHouse/isBlocked")
  public boolean isBlocked() {
    return m_doghousCANRange.getIsDetected().getValue();
  }

  private void funnelStop() {
    setFunnelSpeed(0.0);
  }

  private void funnelGo() {
    setFunnelSpeed(p_funnelSpeed.getValue());
  }

  private void funnelSlow() {
    setFunnelSpeed(0.1);
  }

  private void funnelBackwards() {
    setFunnelSpeed(-1.0);
  }

  private void manipulatorStop() {
    setManipulatorSpeed(0.0);
  }

  private void manipulatorIn() {
    setManipulatorSpeed(p_manipulatorInSpeed.getValue());
  }

  private void manipulatorShoot() {
    setManipulatorSpeed(p_manipulatorShootSpeed.getValue());
  }

  private void manipulatorSlow() {
    setManipulatorSpeed(-0.12);
  }

  private void manipulatorAlgaeSlow() {
    setManipulatorSpeed(0.4);
  }

  private void algaePickup() {
    // setManipulatorSpeed(-0.15);
    m_manipulator.setControl(m_algaePickupRequest);
    funnelBackwards();
  }

  private void algaeShoot() {
    setManipulatorSpeed(1.0);
  }

  private void manipulatorFullSpeed() {
    setManipulatorSpeed(-1.0);
  }

  private void manipulatorL1Speed() {
    setManipulatorSpeed(-0.1);
  }

  private void manipulatorHoldPosition() {
    m_manipulator.setControl(request.withPosition(0.0));
  }

  private void setAlgae() {
    algaeMode = true;
  }

  public boolean isAlgaeMode() {
    return algaeMode;
  }

  public Command stopAllFactory() {
    return new RunCommand(
        () -> {
          funnelStop();
          manipulatorStop();
        },
        this);
  }

  public Command algaePickupFactory() {
    return new RunCommand(
            () -> {
              funnelStop();
              algaePickup();
            },
            this)
        .until(() -> hasCoralDebounced())
        .andThen(
            () -> {
              manipulatorAlgaeSlow();
            });
  }

  public Command setAlgaeSlowFactory() {
    return new RunCommand(() -> manipulatorAlgaeSlow(), this);
  }

  public boolean hasAlgae() {
    return m_manipulator.getSupplyCurrent().getValueAsDouble() > 20.0;
  }

  public Command coralIntakeFactory(BooleanSupplier elevatorDown) {
    return new RunCommand(
        () -> {
          if (!algaeMode) {
            if (!elevatorDown.getAsBoolean() & !isBlocked()) {
              manipulatorStop();
              funnelStop();
              // maybe funnel slow backwards
            } else if (!elevatorDown.getAsBoolean() & isBlocked()) {
              manipulatorSlow();
              funnelSlow();
            } else if (!hasCoral()) {
              manipulatorIn();
              funnelGo();
            } else if (hasCoral() & !isBlocked()) {
              manipulatorHoldPosition();
              funnelStop();
            } else if (isBlocked()) {
              manipulatorSlow();
              funnelGo();
            }
          } else {
            algaePickup();
          }
        },
        this);
  }

  public Command algaeMode() {
    return new RunCommand(
        () -> {
          algaeMode = true;
          if (!hasCoralDebounced()) {
            algaePickup();
            funnelStop();
            m_algaeCaptured = false;
          } else if (hasCoralDebounced()) {
            manipulatorAlgaeSlow();
            funnelStop();
            m_algaeCaptured = true;
          }
        },
        this);
  }

  public Command shootFactory(double delay) {
    return new RunCommand(
            () -> {
              manipulatorShoot();
              algaeMode = false;
            },
            this)
        .withTimeout(delay)
        .andThen(
            () -> {
              manipulatorStop();
            });
  }

  public Command shootFullSpeedFactory(double delay) {
    return new RunCommand(
            () -> {
              manipulatorFullSpeed();
              algaeMode = false;
            },
            this)
        .withTimeout(delay)
        .andThen(
            () -> {
              manipulatorStop();
            });
  }

  public Command shootL1() {
    return new RunCommand(
            () -> {
              manipulatorL1Speed();
              algaeMode = false;
            },
            this)
        .withTimeout(1.0)
        .andThen(
            () -> {
              manipulatorStop();
            });
  }

  public Command shootAlgaeFactory() {
    return new RunCommand(
            () -> {
              algaeShoot();
              algaeMode = false;
            },
            this)
        .withTimeout(1.0);
  }

  public Command setAlgaeModeFactory() {
    return new InstantCommand(() -> setAlgae(), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Doghouse/Funnnel/Current", m_funnel.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Doghouse/Has Coral", hasCoral());
    SmartDashboard.putNumber(
        "Doghouse/Block Distance",
        Units.metersToInches(m_doghousCANRange.getDistance().getValueAsDouble()));
    SmartDashboard.putNumber(
        "Doghouse/Coral Distance",
        Units.metersToInches(m_coralRange.getDistance().getValueAsDouble()));
    SmartDashboard.putBoolean("Doghouse/Is Blocked", isBlocked());
    SmartDashboard.putBoolean("Doghouse/Reef", m_reefRange.getIsDetected().getValue());
    SmartDashboard.putBoolean("isAlgae", algaeMode);
  }
}
