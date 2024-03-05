package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autos.TranslateDistance;
import frc.robot.commands.Vision.VisionTranslate;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TranslateThenShoot extends SequentialCommandGroup{
    
    public TranslateThenShoot(Swerve swerve, Intake intake, Shoot shoot, Vision vision) {
        addCommands(
            // new TranslateDistance(swerve, 1).withTimeout(1), //pose x value is read in meters
            new VisionTranslate(swerve, vision, 0.2, 0),
            new AuxShoot(intake, shoot)
        );
    }
}
