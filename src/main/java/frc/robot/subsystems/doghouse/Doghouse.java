// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.doghouse;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preflib.PrefGroup;
import frc.robot.util.preflib.PrefLib;

public class Doghouse extends SubsystemBase {
  private final TalonFX m_funnel = new TalonFX(Constants.DOGHOUSE_FUNNEL_MOTOR, "rio");
  private final TalonFX m_manipulator =
      new TalonFX(Constants.ELEVATOR_MANIPULATOR_MOTOR, Constants.RIO_CANBUS);
  private final CANrange m_doghousCANRange =
      new CANrange(Constants.DOGHOUSE_CANRANGE, Constants.RIO_CANBUS);
  private final CANrange m_coralRange =
      new CANrange(Constants.CORAL_CANRANGE, Constants.RIO_CANBUS);

  private final PrefGroup m_prefs = PrefLib.getGroup("Doghouse");
  private final PrefGroup m_funnelPrefs = m_prefs.subgroup("Funnel");
  private final PrefGroup m_manipulatorPrefs = m_prefs.subgroup("Manipulator");

  private final DutyCycleOut m_funnelRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut m_manipulatorRequest = new DutyCycleOut(0.0);

  private boolean m_coralCaptured = false;

  public Doghouse() {
    // configure funnel
    TalonFXConfiguration doghouseConfiguration = new TalonFXConfiguration();
    doghouseConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_funnel.getConfigurator().apply(doghouseConfiguration);

    m_funnelPrefs.createCurrentLimitPrefs(m_funnel.getConfigurator()::apply);

    // configure manipulator
    m_manipulatorPrefs.createCurrentLimitPrefs(m_manipulator.getConfigurator()::apply);

    configureCANrange();
  }

  private void configureCANrange() {
    CANrangeConfiguration coralRangecfg = new CANrangeConfiguration();
    CANrangeConfiguration doghouscfg = new CANrangeConfiguration();
    coralRangecfg.FovParams.FOVRangeX = 6.75;
    doghouscfg.FovParams.FOVRangeX = 7.5;
    coralRangecfg.FovParams.FOVRangeY = 6.75;
    doghouscfg.FovParams.FOVRangeY = 27.0;

    doghouscfg.ToFParams.UpdateFrequency = 50;
    coralRangecfg.ToFParams.UpdateFrequency = 50;

    doghouscfg.ProximityParams.ProximityThreshold = 0.5;
    doghouscfg.ProximityParams.ProximityHysteresis = 0.03;
    coralRangecfg.ProximityParams.ProximityThreshold = 0.5;

    coralRangecfg.ProximityParams.MinSignalStrengthForValidMeasurement = 40000;
    doghouscfg.ProximityParams.MinSignalStrengthForValidMeasurement = 10000;

    m_doghousCANRange.getConfigurator().apply(doghouscfg);
    m_coralRange.getConfigurator().apply(coralRangecfg);
  }

  private void setFunnelSpeed(double output) {
    m_funnel.setControl(m_funnelRequest.withOutput(output));
  }

  private void setManipulatorSpeed(double output) {
    m_manipulator.setControl(m_manipulatorRequest.withOutput(output));
  }

  public boolean hasCoral() {
    return m_coralRange.getIsDetected().getValue();
  }

  private boolean isBlocked() {
    return m_doghousCANRange.getIsDetected().getValue();
  }

  private void funnelStop() {
    setFunnelSpeed(0.0);
  }

  private void funnelGo() {
    setFunnelSpeed(m_funnelPrefs.getValue("Speed", 1.));
  }

  private void manipulatorStop() {
    setManipulatorSpeed(0.0);
  }

  private void manipulatorIn() {
    setManipulatorSpeed(m_manipulatorPrefs.getValue("InSpeed", -0.2));
  }

  private void manipulatorShoot() {
    setManipulatorSpeed(m_manipulatorPrefs.getValue("ShootSpeed", -0.3));
  }

  private void manipulatorSlow() {
    setManipulatorSpeed(-0.1);
  }

  private void algaePickup() {
    setManipulatorSpeed(1.0);
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
        this);
  }

  public Command coralIntakeFactory() {
    return new RunCommand(
        () -> {
          if (!hasCoral()) {
            manipulatorIn();
            funnelGo();
            m_coralCaptured = false;
          } else if (m_coralCaptured) {
            manipulatorStop();
            funnelStop();
          } else if (isBlocked()) {
            manipulatorSlow();
            funnelStop();
          } else if (!isBlocked()) {
            manipulatorStop();
            funnelStop();
            m_coralCaptured = true;
          }
        },
        this);
  }

  public Command shootFactory() {
    return new RunCommand(() -> manipulatorShoot(), this)
        .withTimeout(1.0)
        .andThen(
            () -> {
              manipulatorStop();
            });
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
  }
}
