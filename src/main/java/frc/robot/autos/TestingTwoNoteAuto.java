package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakePowerCommand;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Shoot.AimAndShoot;
import frc.robot.commands.Vision.RotateToHeading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TestingTwoNoteAuto extends SequentialCommandGroup{
                /* This needs to be optamized to move faster/use less time for shooting because it uses all 15 seconds */
    
    public TestingTwoNoteAuto(Swerve swerve, Vision vision, Shoot shoot, Intake intake) {
        // addCommands(
        //     new InstantCommand(() -> swerve.zeroHeading()).withTimeout(0.01),
        //     new AimAndShoot(swerve, vision, shoot, intake).withTimeout(3),
        //     new RotateToHeading(swerve, 0).withTimeout(0.5),
        //     new ParallelCommandGroup(
        //     new InstantCommand(() -> swerve.drive(new Translation2d(0.15/ 20, 0), 0, false, true)),
        //     new IntakeSetpointCommand(intake, Constants.IntakeConstants.groundSetpoint),
        //     new IntakePowerCommand(intake, Constants.IntakeConstants.intakeSuckVolts)
        //     ).withTimeout(4),
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> swerve.drive(new Translation2d(-0.15/ 20, 0), 0, false, true)),
        //         new IntakeSetpointCommand(intake, 0),
        //         new IntakePowerCommand(intake, -0.5)
        //     ).withTimeout(4),
        //     new AimAndShoot(swerve, vision, shoot, intake).withTimeout(2)
        // );
    }
}
