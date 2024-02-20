package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class RotateToHeading extends Command{
    private final Swerve swerve;
    private final double setpoint;

    private PIDController rotationPID = new PIDController(0.1, 0, 0);

    public RotateToHeading(Swerve swerve, double setpoint) {
        this.swerve = swerve;
        this.setpoint = setpoint;
        
        rotationPID.setTolerance(0);
        rotationPID.setSetpoint(setpoint);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double rotation = rotationPID.calculate(swerve.getGyroDouble() , 0.1);

        swerve.drive(new Translation2d(), rotation, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
