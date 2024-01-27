// 1/26/24 Kaden
// added magic motion control velocity shooting; deprecated useless commands

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase {
  
  private TalonFX motor1 = new TalonFX(31);
  private TalonFX motor2 = new TalonFX(22);

  // private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.02, 0.02, 0.02);

  public Shoot() {
    motor1.setInverted(false);
    motor2.setInverted(true);
    // pivotMotor.setInverted(false);

    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);
    // pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    motor1.getConfigurator().apply(new TalonFXConfiguration());
    motor2.getConfigurator().apply(new TalonFXConfiguration());
    // pivotMotor.getConfigurator().apply(new TalonFXConfiguration());

    // PIDController shootingPIDController = new PIDController(0.2, 0.002, 0);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = 0.25; //Adds volts to overcome static friction
      slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
      slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = 0.0; 
      slot0Configs.kD = 0.0;

      var motionMagicConfigs = talonFXConfigs.MotionMagic;

      motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 200 rps/s (0.5 seconds to full speed)
      motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

      motor1.getConfigurator().apply(talonFXConfigs);
      motor2.getConfigurator().apply(talonFXConfigs);
  }

  @Deprecated
  public Command RunMotors(double speed) {
    return run(
        () -> {
         setMotorSpeed(speed);
        });
  }

  @Deprecated
  public Command RunMotorVoltage(double rpm) {
    return run(() -> {
      setMotorVolts(RPMToVolts(rpm));
      System.out.println("Motor1 RPM = " + getRPMfromVelocity1() + "Motor2 RPM" + getRPMfromVelocity2());
    });
  }


  public void setMotorSpeed(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  public void setSplitMotorSpeed(double one, double two) {
    motor1.set(one);
    motor2.set(two);
  }

  public void setSplitMotorVolts(double one, double two) {
    motor1.setVoltage(one);
    motor2.setVoltage(two);
  }

  public void setMotorVolts(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  
  public void setMotorVelocity(double rps, boolean wantSlow) {
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);

    //if user wants slower shooting, assigns the acceleration to lower, if not, goes to default acceleration
    double acceleration = wantSlow ? 100 : 0;
    request.Acceleration = acceleration;

    motor1.setControl(request.withVelocity(rps));
    motor2.setControl(request.withVelocity(rps));

  }

  public void brakeMotors() {
    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coastMotors() {
    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);
  }

  public double getRPMfromVelocity1() {
    // return (motor1.getVelocity().getValueAsDouble() * 600) / 2048; //real formula
    return (motor1.getVelocity().getValueAsDouble() * 600) / 2048;
  }

  public double getRPMfromVelocity2() {
    // return (motor2.getVelocity().getValueAsDouble() * 600) / 2048; //real formula
    return (motor2.getVelocity().getValueAsDouble()  * 600) / 2048;
  }

  @Deprecated
  public double RPMToVolts(double TargetRPM) {
    //Formula: VConstant = (AppliedVolts / VelocityatVolts) 
    double velocityConstant = 16 / 4500; //Sample Numbers -> vc = 0.002667
    return velocityConstant * TargetRPM;
  }


  public static double InchesToRPM(double inches) {
    return inches * 7.47 + 3800.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor 1 RPM", getRPMfromVelocity1());
    SmartDashboard.putNumber("Motor 2 RPM", getRPMfromVelocity2());
  }
}