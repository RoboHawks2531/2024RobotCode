// 1/28/24 Kaden
/* This was just made in prep for the actual elevator being put on the robot.
   The skeleton was made based off of the cad design I was shown.

   Learning Note: The best way to define setpoints in a mechanism is using PIDSetpoints and controllers.
   Here, I am using the Motion Magic position control that was added with the PhoenixV6 update
   It should be very similar, but it provides better results and less room for errors 
*/
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{

    private TalonFX leftMotor = new TalonFX(Constants.DeviceConstants.leftElevatorMotor); //adjust to actual ID
    private TalonFX rightMotor = new TalonFX(Constants.DeviceConstants.rightElevatorMotor); //adjust to actual ID

    public Elevator() {
        //according to a ctre worker, ccw is the positive direction for a non-inverted falcon
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        rightMotor.getConfigurator().apply(new TalonFXConfiguration());
        leftMotor.getConfigurator().apply(new TalonFXConfiguration());

        
        // var talonFXConfigs = new TalonFXConfiguration();

        //this was just taken from ctre cause its what we need
        // var slot0Configs = talonFXConfigs.Slot0;
        // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0Configs.kP = 0.15; // A position error of 2.5 rotations results in 12 V output
        // slot0Configs.kI = 0; // no output for integrated error
        // slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

        // // set Motion Magic settings
        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        
        // motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 60; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // leftMotor.getConfigurator().apply(talonFXConfigs);
        // rightMotor.getConfigurator().apply(talonFXConfigs);
    }


    public void moveToSetpoint(double setpoint) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0);

        leftMotor.setControl(request.withPosition(setpoint));
        rightMotor.setControl(request.withPosition(setpoint));
    }

    public void zeroMotorEncoders() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    //this will be used in case the motion magic doesnt work how I want it to
    public void setMotors(double speed1, double speed2) {
        leftMotor.set(-speed1);
        rightMotor.set(speed2);
        // if elevator needs to flip directions, make speed1 positive and make speed2 negative
        // if need to flip, flip the encoders values to
    }

    public double getEncoderLeft() {
        return -leftMotor.getPosition().getValueAsDouble();
    }

    public double getEncoderRight() {
        return rightMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Elevator", getEncoderLeft());
        SmartDashboard.putNumber("Right Elevator", getEncoderRight());
    }

}
