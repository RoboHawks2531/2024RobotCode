// 1/27/24 Kaden
// Created Command, this aims and then fires a note, is still missing some key ingredients

/* NOTE: this can be used by either binding to a button or as an auto command */

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Vision.RotateToTarget;
import frc.robot.commands.Vision.VisionTranslate;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AimAndShoot extends SequentialCommandGroup{

    public AimAndShoot(Swerve swerve, Vision vision, Shoot shoot, Intake intake, Candle candle) {
        addCommands(
            new ParallelCommandGroup(
                // new PulseNote(intake, shoot).withTimeout(0.9),
                // new TranslateThenShoot(swerve, intake, shoot)
                new VisionTranslate(swerve, vision, 0.2, 0)
            ).withTimeout(1),
            new ParallelCommandGroup(
                 new RotateToTarget(swerve, vision, candle),
                 new AuxShoot(intake, shoot)
            )
        );
    }
    
}
