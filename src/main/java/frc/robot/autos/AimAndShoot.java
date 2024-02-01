// 1/27/24 Kaden
// Created Command, this aims and then fires a note, is still missing some key ingredients

/* NOTE: this can be used by either binding to a button or as an auto command */

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotateToTarget;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AimAndShoot extends SequentialCommandGroup{

    private Swerve swerve;
    private Vision vision;
    private Shoot shoot;

    public AimAndShoot(Swerve swerve, Vision vision, Shoot shoot) {
        new ParallelCommandGroup(       //Runs Aiming and 'Reving' at the same time to save time
            new RotateToTarget(swerve, vision), //activates the vision aiming for at most 1.1 seconds
            new RunCommand(() -> shoot.setMotorVelocity(5, false)) //
        ).withTimeout(1.4);
       //TODO: when an index and intake system are added
       // implement and call a command/method that:
       // keep the shooter at its target rpm,
       // bring the intake to its store position,
       // reverse the intake to index the note into shooter.

    }
    
}
