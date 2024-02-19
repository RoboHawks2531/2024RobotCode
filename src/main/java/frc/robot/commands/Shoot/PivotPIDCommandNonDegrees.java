package frc.robot.commands.Shoot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.subsystems.Shoot;

public class PivotPIDCommandNonDegrees extends Command{
    
    private Shoot shoot;
    private double setpoint;

    private PIDController pidController = new PIDController(0.027, 0, 0);

    public PivotPIDCommandNonDegrees(Shoot shoot, double setpoint) {
        this.shoot = shoot;
        this.setpoint = setpoint;

        addRequirements(shoot);

        pidController.setTolerance(0);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(shoot.getPivotEncoder(), setpoint);

        shoot.setPivotMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shoot.setPivotMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
