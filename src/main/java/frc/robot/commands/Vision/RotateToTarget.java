// 1/27/24 Kaden
//created this command


package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;


public class RotateToTarget extends Command{
    
    private Swerve swerve;
    private Vision vision;
    private Candle candle;

    private PIDController rotationPID = new PIDController(0.1, 0, 0.0);

    public RotateToTarget(Swerve swerve, Vision vision, Candle candle) {
        this.swerve = swerve;
        this.vision = vision;
        this.candle = candle;
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
        double rotation = rotationPID.calculate(vision.getYaw(), -2);

        swerve.drive(new Translation2d( //if this doesnt work, revert it back
            -MathUtil.applyDeadband(RobotContainer.driver.getRawAxis(XboxController.Axis.kLeftY.value), Constants.stickDeadband), -MathUtil.applyDeadband(RobotContainer.driver.getRawAxis(XboxController.Axis.kLeftX.value), Constants.stickDeadband)).times(1)
            , rotation, false, true);
        if (vision.hasTarget()) {
            Candle.LEDSegment.MainStrip.setStrobeAnimation(candle.green, 0.3);
        }
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
