package frc.robot.commands.Shoot;

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
                new PivotPIDCommand(shoot, -10),
                new IntakeSetpointCommand(intake, 0)
            ).withTimeout(0.5),
            new ParallelCommandGroup(
                new RevShooter(shoot, 50),
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
