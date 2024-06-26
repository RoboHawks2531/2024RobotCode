package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private TalonFX pivotMotor = new TalonFX(Constants.DeviceConstants.intakePivotMotor);
    private TalonFX powerMotor = new TalonFX(Constants.DeviceConstants.intakePowerMotor);

    private DigitalInput intakeLimitSwitch = new DigitalInput(0);

    public Intake() {
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        powerMotor.setNeutralMode(NeutralModeValue.Brake);

        powerMotor.setInverted(false);

        var talonFXConfigs = new TalonFXConfiguration();

        //this was just taken from ctre cause its what we need
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.09; // An acceleration of 1 rps/s requires 0.01 V output .01
        slot0Configs.kP = 0.15; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds) 60
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        pivotMotor.getConfigurator().apply(talonFXConfigs);
        // powerMotor.getConfigurator().apply(talonFXConfigs);

        var talonFXConfiguration1 = new TalonFXConfiguration();

        talonFXConfiguration1.CurrentLimits.StatorCurrentLimit = 60;
        talonFXConfiguration1.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXConfiguration1.CurrentLimits.SupplyCurrentLimit = 22;
        talonFXConfiguration1.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfiguration1.CurrentLimits.SupplyCurrentThreshold = 25;
        talonFXConfiguration1.CurrentLimits.SupplyTimeThreshold = 0.1;

        powerMotor.getConfigurator().apply(talonFXConfiguration1);
    }


    public void setPowerVelocity(double rpm, boolean wantSlow) {
        final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

        //if user wants slower shooting, assigns the acceleration to lower, if not, goes to default acceleration
        double acceleration = wantSlow ? 100 : 0;
        request.Acceleration = acceleration;

        double velocity = wantSlow ? 0.06 : 0;
        request.Velocity = velocity;

        powerMotor.setControl(request.withVelocity(rpm / 60));
    }

    public void setPowerVolts(double volts) {
        powerMotor.setVoltage(volts);
    }

    public double getPowerVelocity() {
        return powerMotor.getVelocity().getValueAsDouble();
    }

    public void zeroPivotEncoder() {
        pivotMotor.setPosition(0);
    }

    public double getPivotEncoder() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public void setPivotVolts(double volts) {
        pivotMotor.setVoltage(volts);
    }

    public void moveToSetpoint(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);

        pivotMotor.setControl(request.withPosition(setpoint));
    }

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public boolean getLimitSwitch() {
        return !intakeLimitSwitch.get();
    }

    public void setIntakeAndSetpoint(double rpm, double setpoint) {
        double rps = rpm * 60;
        setPowerVelocity(rps, false);
        moveToSetpoint(setpoint);
    }


    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Intake Limit Switch", getLimitSwitch());
        SmartDashboard.putNumber("Intake Pivot Encoder", getPivotEncoder());
        SmartDashboard.putNumber("Power Motor Velocity", getPowerVelocity());
    }
}
