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
            new IntakeSetpointCommand(intake, -3.6).withTimeout(0.5),
            new ParallelCommandGroup(
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexHoldVolts)),
                new InstantCommand(() -> intake.setPowerVolts(5)) 
            ).withTimeout(0.2),
            new ParallelCommandGroup(
                new InstantCommand(() -> shoot.setIndexMotorVolts(-4)),
                new InstantCommand(() -> intake.setPowerVolts(-3)) 
            ).withTimeout(0.2)
        );
    }
}
