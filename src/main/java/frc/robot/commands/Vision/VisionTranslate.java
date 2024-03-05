package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class VisionTranslate extends Command {
    private Swerve swerve;
    private Vision vision;

    private PIDController translationPID = new PIDController(0.1, 0, 0);
    private double xDistance;
    private double yDistance;

    public VisionTranslate(Swerve swerve, Vision vision, double xDistance, double yDistance) {
        this.swerve = swerve;
        this.vision = vision;
        this.xDistance = xDistance;
        this.yDistance = yDistance;

        // addRequirements(swerve);
        
        translationPID.setTolerance(0.1);
        // translationPID.setSetpoint(distance);
    }

    @Override
    public void initialize() {
        // translationPID.reset();
    }

    @Override
    public void execute() {
        double speedX = translationPID.calculate(vision.getDistanceMethod(), xDistance);
        double speedY = translationPID.calculate(vision.getYaw(), yDistance);

        swerve.drive(new Translation2d(speedX, speedY), 0, true, true);

        // swerve.setChassisSpeeds(swerve.getChassisSpeeds());
    }
}
