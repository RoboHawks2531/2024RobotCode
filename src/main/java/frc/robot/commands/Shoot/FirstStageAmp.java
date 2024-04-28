package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class FirstStageAmp extends SequentialCommandGroup{
    
    public FirstStageAmp(Shoot shoot, Intake intake) {
        addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> intake.setPowerVolts(6)),
            // new InstantCommand(() -> intake.setPowerVelocity(Constants.IntakeConstants.intakeSpitVelocity, false)),
            new InstantCommand(() -> shoot.setIndexMotorVolts(8))).withTimeout(1.5),
            new InstantCommand(() -> shoot.setMotorVelocity(-1, false)), // TODO: remove this if it shreds a note
            // new InstantCommand(() -> intake.setPowerVolts(0)),
            new WaitCommand(0.4),
            new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotAmp));
    }
}
