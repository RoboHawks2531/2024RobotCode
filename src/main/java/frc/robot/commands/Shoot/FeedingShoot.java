package frc.robot.commands.Shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Vision;

public class FeedingShoot extends SequentialCommandGroup{
    
    public FeedingShoot(Shoot shoot, Intake intake, Vision vision) {
        addCommands(
            new ParallelCommandGroup(
    
                new IntakeSetpointCommand(intake, -2),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)),
                new PivotPIDCommandNonDegrees(shoot, 8),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(1.3),
            new ParallelCommandGroup(
                new IntakeSetpointCommand(intake, -2),
                new PivotPIDCommandNonDegrees(shoot, 8),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new InstantCommand(() -> intake.setPowerVolts(10)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(14))
            )    
        );
    }
}
