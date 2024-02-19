package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class IndexNote extends ParallelCommandGroup{
    
    public IndexNote(Intake intake, Shoot shoot) {
        addCommands(
            new InstantCommand(() -> intake.setPowerVolts(10)),
            // new InstantCommand(() -> intake.setPowerVelocity(Constants.IntakeConstants.intakeSpitVelocity, false)),
            new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts))
            // new InstantCommand(() -> shoot.setIndexMotorVelocity(2000))
        );
    }
}
