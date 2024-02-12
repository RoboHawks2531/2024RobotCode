package frc.robot.commands.Shoot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.subsystems.Shoot;

public class PivotPIDCommand extends Command{
    
    private Shoot shoot;
    private double setpointDegrees;

    private PIDController pidController = new PIDController(0.015, 0, 0);

    public PivotPIDCommand(Shoot shoot, double setpointDegrees) {
        this.shoot = shoot;
        this.setpointDegrees = setpointDegrees;

        addRequirements(shoot);

        pidController.setTolerance(0);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(Conversions.falconToDegrees(shoot.getPivotEncoder(), .008), setpointDegrees);

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
