//incase we switch back to limelights software

package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class LimelightAim extends Command{

    private Swerve swerve;
    private Limelight limelight;
    private boolean foundValidTarget = false;

    private PIDController pidController = new PIDController(0.1, 0, 0);

    private double maxPower = 0.5;

    public LimelightAim(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;

        pidController.setTolerance(1);
    }

    @Override
    public void initialize() {
        pidController.reset();
        limelight.ensureEnabled();
        foundValidTarget = false;
    }

    @Override
    public void execute() {
        if (!limelight.isReady()) return;

        if (limelight.hasValidTarget()) {
            foundValidTarget = true;
            pidController.setSetpoint(Math.toDegrees(Math.atan2(-2, limelight.getDistance())));
            double rotation = MathUtil.clamp(pidController.calculate(limelight.getX()), -maxPower, maxPower);
            swerve.drive(new Translation2d(), -rotation, false, false);
        }
    }
    
}
