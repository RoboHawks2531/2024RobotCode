package frc.robot.commands.Shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakePivotAutomatically;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Vision.RotateToTarget;
import frc.robot.commands.Vision.VisionTranslate;
// import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TrapShoot extends SequentialCommandGroup{
    private double intakeMult = -2;

    public TrapShoot(Swerve swerve, Vision vision, Shoot shoot, Intake intake) {
        addCommands(
            new ParallelCommandGroup(
                // new RotateToTarget(swerve, vision),
                // new VisionTranslate(swerve, vision, 1.5, 0),
                // new IntakeSetpointCommand(intake, MathUtil.clamp(-vision.getDistanceMethod() * 7, -10, 0)),
                new IntakeSetpointCommand(intake, -5),  //TODO:josiah change this number
                // new IntakePivotAutomatically(intake, vision),
                // new PivotShootVertically(shoot, vision),
                new PivotPIDCommandNonDegrees(shoot, 7.5),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)),
                new RevShooter(shoot, 2000)
            ).withTimeout(1.3),
            new ParallelCommandGroup(
                new IntakeSetpointCommand(intake, -5), //TODO:josiah change this number
                // new IntakePivotAutomatically(intake, vision),
                // new PivotShootVertically(shoot, vision),
                new PivotPIDCommandNonDegrees(shoot, 7.5),
                new RevShooter(shoot, 2000),
                new InstantCommand(() -> intake.setPowerVolts(10)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(14))
            )    
        );
    }
    
}
