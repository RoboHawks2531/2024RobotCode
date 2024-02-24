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
                new IntakeSetpointCommand(intake, 0), //pulsing puts the intake here anyways
                new InstantCommand(() -> intake.setPowerVolts(-1)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)), //sped up because david said so
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(3),
            new ParallelCommandGroup(
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new IndexNote(intake, shoot)
            )
        );
    }
}


            