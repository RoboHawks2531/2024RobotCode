// 1/27/24 Kaden
//created this command


package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;


public class RotateToTarget extends Command{
    
    private Swerve swerve;
    private Vision vision;

    private PIDController rotationPID = new PIDController(0.02, 0.002, 0);

    public RotateToTarget(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);

        rotationPID.setSetpoint(0);
        rotationPID.setTolerance(0.1);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double rotSpeed;  

        if (!rotationPID.atSetpoint()) {
            rotSpeed = rotationPID.calculate(vision.getYaw());
        } rotSpeed = 0;
        
        swerve.drive(null, rotSpeed, false, true);
    }   

    @Override
    public void end(boolean interrupted) {
        swerve.drive(null, 0, false, false);
    }
}
