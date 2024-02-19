package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class AuxShoot extends SequentialCommandGroup{
    
    public AuxShoot(Intake intake, Shoot shoot) {
        addCommands(
            new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore), // re-add this if we start using the pivot again
                new IntakeSetpointCommand(intake, -3.6),
                new InstantCommand(() -> shoot.setIndexMotorVolts(8)),
                // new InstantCommand(() -> shoot.setIndexMotorVelocity(Constants.ShootingConstants.indexFeedVelocity)),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(1.5),
            new ParallelCommandGroup(
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new IndexNote(intake, shoot)
            )
        );
    }
}
