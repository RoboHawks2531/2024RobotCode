package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class BabyShoot extends SequentialCommandGroup{
    
    public BabyShoot(Intake intake, Shoot shoot) {
        addCommands(
            new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore), // re-add this if we start using the pivot again
                new IntakeSetpointCommand(intake, Constants.IntakeConstants.indexFeedingSetpoint), //pulsing puts the intake here anyways
                new InstantCommand(() -> shoot.setIndexMotorVolts(3)), //sped up because david said so
                new RevShooter(shoot, 1000)
            ).withTimeout(.5),
            new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore),
                // new PivotPIDCommandNonDegrees(shoot, -2.5), //why god
                new IntakeSetpointCommand(intake, Constants.IntakeConstants.indexFeedingSetpoint),
                new RevShooter(shoot, 1000),
                new InstantCommand(() -> intake.setPowerVolts(4)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(3))
                // new IndexNote(intake, shoot)
            )
        );
    }
}            