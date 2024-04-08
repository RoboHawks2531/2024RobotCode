package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class FeedingShoot extends SequentialCommandGroup{
    
    public FeedingShoot(Shoot shoot, Intake intake) {
        addCommands(
            new ParallelCommandGroup(
                new IndexHold(intake, shoot),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new PivotPIDCommandNonDegrees(shoot, -10)
            ).withTimeout(1),
            new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, -10),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts))
            )    
        );
    }
}
