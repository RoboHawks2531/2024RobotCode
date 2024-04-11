package frc.robot.commands.Shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Vision.RotateToTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Vision;

public class DistanceShoot extends SequentialCommandGroup{
    
    public DistanceShoot(Intake intake, Shoot shoot, Vision vision) {
        addCommands(
            new ParallelCommandGroup(
                new IntakeSetpointCommand(intake, MathUtil.clamp(-vision.getDistanceMethod() * 1.2, -10, 0)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)),
                new PivotPIDCommandNonDegrees(shoot, MathUtil.clamp(vision.getDistanceMethod() * 4.1, 0.5, 15)),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM)
            ).withTimeout(1.3),
            new ParallelCommandGroup(
                new IntakeSetpointCommand(intake, MathUtil.clamp(-vision.getDistanceMethod() * 1.2, -10, 0)),
                new PivotPIDCommandNonDegrees(shoot, 8),
                new RevShooter(shoot, Constants.ShootingConstants.targetShootingRPM),
                new InstantCommand(() -> intake.setPowerVolts(10)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(14))
            )    
        );
    }
}


            