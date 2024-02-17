package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class AuxShoot extends SequentialCommandGroup{
    
    public AuxShoot(Intake intake, Shoot shoot) {
        addCommands(
            new ParallelCommandGroup(
                // new PivotPIDCommand(shoot, 0),
                new IntakeSetpointCommand(intake, -3.2),
                new InstantCommand(() -> shoot.setIndexMotorVolts(8)),
                new RevShooter(shoot, -6800)
            ).withTimeout(1.5),
            new ParallelCommandGroup(
                new RevShooter(shoot, -6800),
                new IndexNote(intake, shoot)
            )
        );
        
        
        
        
        
        // addCommands(
        //     new ParallelCommandGroup(
        //     new RunCommand(() -> intake.moveToSetpoint(0)),
        //     new RevShooter(shoot, 0)
        // )).withTimeout(.5),
        // new ParallelCommandGroup(
        //     new RunCommand(() -> shoot.setMotorVelocity(Constants.ShootingConstants.targetShootingRPM, false)),
        //     new IndexNote(intake, shoot)
        // ));
    }
}
