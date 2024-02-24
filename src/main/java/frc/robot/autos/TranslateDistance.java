package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TranslateDistance extends Command{
    private Swerve swerve;
    private double distance;

    private PIDController translationPID = new PIDController(0.1, 0, 0);
    
    public TranslateDistance(Swerve swerve, double distance) {
        this.swerve = swerve;
        this.distance = distance;

        translationPID.setTolerance(0.1);
        translationPID.setSetpoint(distance);
    }

    @Override
    public void initialize() {
        // swerve.setPose(new Pose2d());
    }

    @Override
    public void execute() {
        double speed = translationPID.calculate(swerve.getPose().getX(), distance);
        swerve.drive(new Translation2d(speed,0), 0, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false, false);
    }
}
