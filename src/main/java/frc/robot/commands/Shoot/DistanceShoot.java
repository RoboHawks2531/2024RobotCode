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

public class DistanceShoot extends SequentialCommandGroup{
    
    public DistanceShoot(Intake intake, Shoot shoot) {
        addCommands(
            // new ParallelCommandGroup(
            //     new InstantCommand(() -> shoot.setIndexMotorVolts(4)),
            //     new InstantCommand(() -> intake.setPowerVolts(5)) 
            // ).withTimeout(0.5),
            new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotDistanceShooting), // re-add this if we start using the pivot again
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(1.5),
            new ParallelCommandGroup(
                // new IntakeSetpointCommand(intake, Constants.IntakeConstants.indexFeedingSetpoint),
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotDistanceShooting), // re-add this if we start using the pivot again
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new InstantCommand(() -> shoot.setIndexMotorVolts(12))
                // new IndexNote(intake, shoot)
            )
        );
    }
}


            