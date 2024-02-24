package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot.AuxShoot;
import frc.robot.commands.Shoot.ResetShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;

public class ShootThenBackUp extends SequentialCommandGroup{

    public ShootThenBackUp(Swerve s_Swerve, Shoot shoot, Intake intake) {
        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(new Pose2d(0,0, new Rotation2d(0)))),
            new AuxShoot(intake, shoot).withTimeout(2),
            new ResetShooter(intake, shoot),
            // new InstantCommand(() -> s_Swerve.drive(new Translation2d(0.15/ 20,0), 0, false, true))
            new RunCommand(() -> s_Swerve.drive(new Translation2d(0.15/ 20,0), 0, false, true)).withTimeout(3)
            // new TranslateDistance(s_Swerve, 2)
        );
    }
    
}
