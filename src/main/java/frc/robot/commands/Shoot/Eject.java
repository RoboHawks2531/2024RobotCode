package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class Eject extends SequentialCommandGroup{
    
    public Eject(Intake intake, Shoot shoot) {
        addCommands(
            new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore), // re-add this if we start using the pivot again
                // new PivotPIDCommandNonDegrees(shoot, -2.5), //why must god look here and say yeah
                new IntakeSetpointCommand(intake, Constants.IntakeConstants.indexFeedingSetpoint), //pulsing puts the intake here anyways
                // new InstantCommand(() -> intake.setPowerVolts(-2)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)), //sped up because david said so
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(.1),
            new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore),
                // new PivotPIDCommandNonDegrees(shoot, -2.5), //why god
                new IntakeSetpointCommand(intake, Constants.IntakeConstants.indexFeedingSetpoint),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                // new InstantCommand(() -> shoot.setIndexMotorVolts(12))
                new IndexNote(intake, shoot)
            )
        );
    }
}            