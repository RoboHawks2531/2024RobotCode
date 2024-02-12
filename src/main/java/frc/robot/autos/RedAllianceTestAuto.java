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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.Vision.RotateToTarget;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RedAllianceTestAuto extends SequentialCommandGroup{
    
    public RedAllianceTestAuto(Swerve swerve, Vision vision) {
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory speakerToTopNotTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(15.15, 5.57, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of( new Translation2d(14.46, 6.28) ,new Translation2d(13.66, 7.02), new Translation2d(15.15, 5.57), new Translation2d(14.66, 5.57), new Translation2d(13.66, 5.57),  new Translation2d(15.10, 5.57)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(15.15, 5.57, new Rotation2d(0)),
                config
                );

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                speakerToTopNotTrajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0.01),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0.01),
                thetaController,
                swerve::setModuleStates,
                swerve);


        addCommands(
            new InstantCommand(() -> swerve.setPose(speakerToTopNotTrajectory.getInitialPose())),
            swerveControllerCommand1, //this should drive to the first note, then allow for other commands to be ran alongside it with a parallel group
            new RotateToTarget(swerve, vision)
        );
    }
}
