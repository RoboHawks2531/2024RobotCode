package frc.robot.commands.Vision;

import java.util.Set;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TagToPoseTrajectoryGenerator extends SequentialCommandGroup{
    
    Swerve swerve;
    Vision vision;
    PhotonTrackedTarget targetToUse;

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    Transform3d tagToGoal;

    private int kIDtoChase;

    public TagToPoseTrajectoryGenerator(Swerve swerve, Vision vision, int kIDtoChase, double frontOffsetInches) {
    this.vision = vision;
    this.swerve = swerve;
    this.kIDtoChase = kIDtoChase;
    this.tagToGoal = new Transform3d(new Translation3d(Units.inchesToMeters(frontOffsetInches), 0, 0),
            new Rotation3d(0, 0, Math.PI));

    addRequirements(vision, swerve);
    addCommands(new DeferredCommand(() -> getCommand(), Set.of(swerve, vision)), new InstantCommand(() -> swerve.stopModules()));
    }

    public Command getCommand() {

        /* 
            Always use the blue alliance tags when calling, they will be auto converted

            Speaker Tags
           ID 7 = center blue speaker Tag
           ID 4 = center red speaker Tag

            Amp Tags
           ID 6 = Blue Amp Tag
           ID 5 = Red Amp Tag

           ID 14, 15, 16 = Blue Stage Tags
           ID 11, 12, 13 = Red Stage Tags
        */
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && kIDtoChase == 7){
        //     kIDtoChase = 4;
        // } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && kIDtoChase == 6){
        //     kIDtoChase = 5;
        // } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && kIDtoChase == 14){
        //     kIDtoChase = 11;
        // } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && kIDtoChase == 15){
        //     kIDtoChase = 12;
        // } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && kIDtoChase == 16){
        //     kIDtoChase = 13;
        // }
     

        var robotPose2d = swerve.getPose();

        var robotPose3d = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0,
                                new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

        var result = vision.getCamera().getLatestResult();

        if (result.hasTargets() == false) {
            return new InstantCommand();
        } else {
            try {
                // var allTargets = result.getTargets();
                //     for (PhotonTrackedTarget target : allTargets) {
                //         if (target.getFiducialId() == kIDtoChase) {
                //                 targetToUse = target;
                //         }
                // }
                PhotonTrackedTarget target = result.getBestTarget();

                if (target.getFiducialId() == kIDtoChase) {
                    targetToUse = target;
                } else {
                    return new InstantCommand();
                }

        // this doesn't really do anything unless "always do single target estimation is
        // checked -> by deafulat, returns -1"
        if (targetToUse.getPoseAmbiguity() >= 0.2) {
            return new InstantCommand();
        }

        // System.out.println("ID: " + targetToUse.getFiducialId() + " ambig = "
        // + targetToUse.getPoseAmbiguity());

        // Get the transformation from the camera to the tag
        var camToTarget = targetToUse.getBestCameraToTarget();

        // Transform the robot's pose to find the tag's pose
        var cameraPose = robotPose3d.transformBy(VisionConstants.robotToCam);
        var targetPose = cameraPose.transformBy(camToTarget);

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

            return AutoBuilder.pathfindToPose(goalPose, new PathConstraints(
                    3.0, 2,
                    Units.degreesToRadians(540), Units.degreesToRadians(720)), 0);

            }
             catch (NullPointerException ex) {
                ex.printStackTrace();
                    return new InstantCommand();
            }
        }
    }
    
}
