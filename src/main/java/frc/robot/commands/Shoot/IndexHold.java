package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class IndexHold extends SequentialCommandGroup{
    
    public IndexHold(Intake intake, Shoot shoot) {
        addCommands(
            new InstantCommand(() -> intake.setPowerVolts(2)),
            new InstantCommand(() -> shoot.setIndexMotorVolts(3))
        );
    }
}
