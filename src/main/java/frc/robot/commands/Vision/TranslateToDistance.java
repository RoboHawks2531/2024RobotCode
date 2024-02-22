package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TranslateToDistance extends Command {
    private Swerve swerve;
    private Vision vision;

    private PIDController translationPID = new PIDController(0.1, 0, 0);
    private double distance;

    public TranslateToDistance(Swerve swerve, Vision vision, double distance) {
        this.swerve = swerve;
        this.vision = vision;
        // addRequirements(swerve);
        
        translationPID.setTolerance(0.1);
        translationPID.setSetpoint(distance);
    }

    @Override
    public void initialize() {
        translationPID.reset();
    }

    @Override
    public void execute() {
        double speed = translationPID.calculate(vision.getDistanceMethod(), distance);

        swerve.drive(new Translation2d(speed, 0), 0, false, true);
    }
}
