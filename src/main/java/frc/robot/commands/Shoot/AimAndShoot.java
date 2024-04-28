// 1/27/24 Kaden
// Created Command, this aims and then fires a note, is still missing some key ingredients

/* NOTE: this can be used by either binding to a button or as an auto command */

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

public class AimAndShoot extends SequentialCommandGroup{
    private double intakeMult = -2;

    public AimAndShoot(Swerve swerve, Vision vision, Shoot shoot, Intake intake) {
        addCommands(
            new ParallelCommandGroup(
                // new RotateToTarget(swerve, vision),
                // new IntakeSetpointCommand(intake, MathUtil.clamp(-vision.getDistanceMethod() * 7, -10, 0)),
                new IntakeSetpointCommand(intake, MathUtil.clamp(-vision.getDistanceMethod() + intakeMult, -10, 0)),
                // new IntakePivotAutomatically(intake, vision),
                new PivotShootVertically(shoot, vision),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(1.3),
            new ParallelCommandGroup(
                new IntakeSetpointCommand(intake, MathUtil.clamp(-vision.getDistanceMethod() + intakeMult, -10, 0)),
                // new IntakePivotAutomatically(intake, vision),
                new PivotShootVertically(shoot, vision),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new InstantCommand(() -> intake.setPowerVolts(10)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(14))
            )    
        );
    }
    
}
