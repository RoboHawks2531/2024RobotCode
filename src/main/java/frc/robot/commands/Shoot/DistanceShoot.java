package frc.robot.commands.Shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.math.Conversions;
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
                new IntakeSetpointCommand(intake, 0),
                // new IntakeSetpointCommand(intake, -8),
                new InstantCommand(() -> shoot.setIndexMotorVolts(Constants.ShootingConstants.indexFeedVolts)),
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore),
                new RevShooter(shoot, 2500)
            ).withTimeout(1.0),
            new ParallelCommandGroup(
                // new IntakeSetpointCommand(intake, MathUtil.clamp(-vision.getDistanceMethod() * 3.5, -10, 0)),
                // new IntakeSetpointCommand(intake, -8),
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore),
                new IntakeSetpointCommand(intake, 0),
                new RevShooter(shoot, 2500),
                new InstantCommand(() -> intake.setPowerVolts(10)),
                new InstantCommand(() -> shoot.setIndexMotorVolts(14))
            )    
        );
    }
}


            