// 1/27/24 Kaden
//created this command


package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;


public class RotateToTarget extends Command{
    
    private Swerve swerve;
    private Vision vision;

    private PIDController rotationPID = new PIDController(0.1, 0, 0);

    public RotateToTarget(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        // addRequirements(swerve);

        // rotationPID.setSetpoint(0);
        rotationPID.setTolerance(0.1);
    }

    @Override
    public void initialize() {
        // rotationPID.reset();
    }

    @Override
    public void execute() {
        // vision.setCameraLEDS(true); // only here for funsies
        double rotation = rotationPID.calculate(vision.getYaw(), 0.1);

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
