package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autos.TranslateDistance;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;

public class TranslateThenShoot extends SequentialCommandGroup{
    
    public TranslateThenShoot(Swerve swerve, Intake intake, Shoot shoot) {
        addCommands(
            new ParallelCommandGroup(
                new TranslateDistance(swerve, 1), //pose x value is read in meters
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore), // re-add this if we start using the pivot again
                // new IntakeSetpointCommand(intake, -3.6), //pulsing puts the intake here anyways
                new InstantCommand(() -> intake.setPowerVolts(-1)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)), //sped up because david said so
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(1.5),
            new ParallelCommandGroup(
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new IndexNote(intake, shoot)
            )
        );
    }
}
