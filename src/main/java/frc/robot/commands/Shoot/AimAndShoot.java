// 1/27/24 Kaden
// Created Command, this aims and then fires a note, is still missing some key ingredients

/* NOTE: this can be used by either binding to a button or as an auto command */

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Vision.RotateToTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AimAndShoot extends SequentialCommandGroup{

    // private Swerve swerve;
    // private Vision vision;
    // private Shoot shoot;

    public AimAndShoot(Swerve swerve, Vision vision, Shoot shoot, Intake intake) {
        new ParallelRaceGroup(
            new RunCommand(() -> intake.moveToSetpoint(0)),
            new RotateToTarget(swerve, vision), //activates the vision aiming for at most 0.5 seconds
            new RunCommand(() -> shoot.setMotorVelocity(Constants.ShootingConstants.targetShootingRPM, false))
        ).withTimeout(.5);
        // new ParallelCommandGroup(
        //     new RunCommand(() -> shoot.setMotorVelocity(Constants.ShootingConstants.targetShootingRPM, false)) //Runs Aiming and 'Reving' at the same time to save time
        // ).withTimeout(.8);
        new ParallelCommandGroup(
            new RunCommand(() -> shoot.setMotorVelocity(Constants.ShootingConstants.targetShootingRPM, false)),
            new RunCommand(() -> shoot.setIndexMotorVolts(2)),
            new RunCommand(() -> intake.setPowerVelocity(-3, false))
        );
    }
    
}