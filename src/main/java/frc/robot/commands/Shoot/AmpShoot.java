package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class AmpShoot extends SequentialCommandGroup {

    public AmpShoot(Shoot shoot, Intake intake) {
        
        addCommands(
          // new IndexNote(intake, shoot).withTimeout(0.2),
          new ParallelCommandGroup(
            new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexHoldVolts)),
            new InstantCommand(() -> intake.setPowerVolts(5)) 
          ).withTimeout(0.2),
        //   new PivotPIDCommand(shoot, 30) //this is going to have to be tuned, if its being wacky use other PID Command
          new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotAmp).withTimeout(0.7),
          new ParallelCommandGroup(
            new RevShooter(shoot, Constants.ShootingConstants.targetShootingAmpTarget),
            new ParallelCommandGroup(
              new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexHoldVolts)),
              new InstantCommand(() -> intake.setPowerVolts(3))
            )
            // new IndexNote(intake, shoot).withTimeout(0.2)
          ).withTimeout(1.5)
        );
    }
}
