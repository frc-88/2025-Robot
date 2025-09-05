package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class GroundIntake extends SubsystemBase{
    private final TalonFX m_pivot_motor = new TalonFX(Constants.INTAKE_PIVOT_MOTOR, Constants.RIO_CANBUS);
    private final TalonFX m_roller_motor = new TalonFX(Constants.INTAKE_ROLLER_MOTOR, Constants.RIO_CANBUS);
    //private final DoublePreferenceConstant p_intakeSpeed = new DoublePreferenceConstant("GroundIntake/PivotMotor/IntakeSpeed", );
    private final DoublePreferenceConstant p_pivotMotorCurrentLimit = new DoublePreferenceConstant("GroundIntake/PivotMotor/CurrentLimit", 60);
    private final DoublePreferenceConstant p_rollerMotorCurrentLimit = new DoublePreferenceConstant("GroundIntake/RollerMotor/CurrentLimit", 60);
    private final MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);
    private final PIDPreferenceConstants intakePID =
        new PIDPreferenceConstants("GroundIntake/PivotMotor/PID", 1, 0, 0, 0.12, 0, 0, 0, 0);
    private final DoublePreferenceConstant p_intakeMaxVelocity =
        new DoublePreferenceConstant("GroundIntake/PivotMotor/MotionMagicVelocity", 40.0);
    private final DoublePreferenceConstant p_intakeMaxAcceleration =
        new DoublePreferenceConstant("GroundIntake/PivotMotor/MotionMagicAcceleration", 80.0);
    private final DoublePreferenceConstant p_intakeJerk =
        new DoublePreferenceConstant("GroundIntake/PivotMotor/MotionMagicJerk", 0.0);
    private final DutyCycleOut m_rollerRequest = new DutyCycleOut(0.0);

public GroundIntake() {
    TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration();
    pivotMotorConfiguration.CurrentLimits.SupplyCurrentLimit = p_pivotMotorCurrentLimit.getValue();
    pivotMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_pivot_motor.getConfigurator().apply(pivotMotorConfiguration);

    TalonFXConfiguration rollerMotorConfiguration = new TalonFXConfiguration();
    rollerMotorConfiguration.CurrentLimits.SupplyCurrentLimit = p_rollerMotorCurrentLimit.getValue();
    rollerMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_roller_motor.getConfigurator().apply(rollerMotorConfiguration);
    }

private void intakeSetAngle(double angle) {
        m_pivot_motor.setControl(motionmagicrequest.withPosition(angle / Constants.INTAKE_ROTATIONS_TO_DEGREES));
      }

private void intakeGoToGround() {
    intakeSetAngle(90);
};

private void intakeRollerFast() {
    setRollerSpeed(.8);
};

private void setRollerSpeed(double output, boolean slowRamp) {
    OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
    config.DutyCycleOpenLoopRampPeriod = slowRamp ? .2 : 0;
    m_roller_motor.getConfigurator().apply(config);
    m_roller_motor.setControl(m_rollerRequest.withOutput(output));
}

private void setRollerSpeed(double output) {
    setRollerSpeed(output, false);
};

public Command groundIntakeFactory() {
    return new RunCommand(() -> {
        intakeGoToGround();
        intakeRollerFast();
    }, this
    );
    }
}