package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Vision.RotateToTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Vision;

public class DistanceShoot extends SequentialCommandGroup{
    
    public DistanceShoot(Intake intake, Shoot shoot, Vision vision) {
        addCommands(
            // new ParallelCommandGroup(
            //     new IndexHold(intake, shoot)
            // ).withTimeout(0.2),
            new ParallelCommandGroup(
                // new IndexHold(intake, shoot),
                // new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                // new InstantCommand(() -> intake.setPowerVolts(10)),
                new IntakeSetpointCommand(intake, -5),
            // new InstantCommand(() -> intake.setPowerVelocity(Constants.IntakeConstants.intakeSpitVelocity, false)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)),
                // new PivotPIDCommandNonDegrees(shoot, 10),
                new PivotShootVertically(shoot, vision),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(1.5),
            new ParallelCommandGroup(
                new PivotShootVertically(shoot, vision),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new InstantCommand(() -> shoot.setIndexMotorVolts(12)),
                new InstantCommand(() -> intake.setPowerVolts(10))
            )
        );
    }
}


            