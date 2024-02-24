package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.Shoot.AuxShoot;
import frc.robot.commands.Shoot.ResetShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;

public class ShootThenBackUp extends SequentialCommandGroup{

    public ShootThenBackUp(Swerve swerve, Shoot shoot, Intake intake) {
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory speakerToTopNotTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),

                List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                speakerToTopNotTrajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);
        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(() -> shoot.zeroPivotEncoder()),
                new InstantCommand(() -> intake.zeroPivotEncoder())
            ),
            new InstantCommand(() -> swerve.setPose(speakerToTopNotTrajectory.getInitialPose())),
            new AuxShoot(intake, shoot).withTimeout(2),
            swerveControllerCommand1
        );
    }
    
}






// new InstantCommand(() -> s_Swerve.setPose(new Pose2d(0,0, new Rotation2d(0)))).withTimeout(0.1),
            // new AuxShoot(intake, shoot).withTimeout(2),
            // new ResetShooter(intake, shoot),
            // new InstantCommand(() -> s_Swerve.drive(new Translation2d(0.15/ 20,0), 0, false, true))
            // new RunCommand(() -> s_Swerve.drive(new Translation2d(0.15 / 20,0), 0, true, true))
            // new TranslateDistance(s_Swerve, 2)
