package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class PulseNote extends SequentialCommandGroup{
    
    public PulseNote(Intake intake, Shoot shoot) {
        addCommands(
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.indexFeedingSetpoint).withTimeout(0.5),
            new ParallelCommandGroup(
                new InstantCommand(() -> shoot.setIndexMotorVolts(3)),
                new InstantCommand(() -> intake.setPowerVolts(2)) 
            ).withTimeout(1),
            new ParallelCommandGroup(
                new InstantCommand(() -> shoot.setIndexMotorVolts(-3)),
                new InstantCommand(() -> intake.setPowerVolts(-4)) //normal values
                // new InstantCommand(() -> shoot.setIndexMotorVolts(3)),
                // new InstantCommand(() -> intake.setPowerVolts(4)) 
            ).withTimeout(1)
        );
    }
}
