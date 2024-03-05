// 1/27/24 Kaden
// added getPitch and getYaw methods to aid vision aiming

// 1/31/24 Kaden
// Fixed limelight so that it actually works

package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.rotationsToDegrees;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    private Swerve swerve;

    private PhotonCamera limelight = new PhotonCamera("2531Limelight");
    private PhotonCamera arduCam = new PhotonCamera("ArduinoCam");

    double cameraHeight = Units.inchesToMeters(4); //how hight is camera off the ground?
    double targetHeight = Units.inchesToMeters(53.88); //how high is the target off the ground(all april tags are the same)
    double cameraPitchRadians = Units.degreesToRadians(0); //what is the cameras angle from level?
    double goalRangeMeters = Units.feetToMeters(3); //target goal distance for getting in range of a target

    private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.2), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, arduCam, robotToCam); //this is not being used

    public Vision() {

    }

    public double getDistanceMethod() {
        // var result = limelight.getLatestResult();
        var result = arduCam.getLatestResult();

        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight, targetHeight, cameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return 0;
    }

    // public Pose3d getPose3d() {
    //     // var result = limelight.getLatestResult();
    //     var result = arduCam.getLatestResult();

    //     if (result.hasTargets()) {
    //         return PhotonUtils.estimateFieldToRobotAprilTag(
    //             result.getBestTarget().getBestCameraToTarget(),
    //             aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId()),
    //             robotToCam);
    //     }
    //     return new Pose3d();
    // }

    public Pose2d getPose2d() {
        var result = arduCam.getLatestResult();

        // if (result.hasTargets()) {
        //     return PhotonUtils.estimateFieldToRobot(
        //         cameraHeight,
        //         targetHeight, 
        //         cameraPitchRadians, 
        //         getPitch(), 
        //         Rotation2d.fromDegrees(-getYaw()), 
        //         swerve.getHeading(), 
        //         new Pose2d(1,1, new Rotation2d(0,0)), 
        //         robotToCam
        //     );
        // }
        return new Pose2d();
    }

    public Translation2d distanceToTranslate(double translation) {
        var result = arduCam.getLatestResult();

        if (result.hasTargets()) {
            return PhotonUtils.estimateCameraToTargetTranslation(translation, Rotation2d.fromDegrees(-getYaw()));
        }
        return new Translation2d(); //this will do nothing if no april tag is detected
    }

    public double getYaw() {
        // limelight.setLED(VisionLEDMode.kOn);
        // var result = limelight.getLatestResult();
        var result = arduCam.getLatestResult();

        // List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        
        if (result.hasTargets()) {
            // targets.add(result.getBestTarget());
            // return targets.get(0).getYaw(); //this will hopefully filter the multitargets and pick one to aim at
            return target.getYaw(); //This should be the one to take the better target and go to that

            // return result.getBestTarget().getYaw();
        }
        return 0;
    }

    public void takeSnapshot() {
        // var result = limelight.getLatestResult();
        var result = arduCam.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        
        if (result.hasTargets()) {
            limelight.takeOutputSnapshot();
        }
    }

    @Deprecated //this does nothing
    public void setCameraLEDS(boolean on) {
        if (on) {
            limelight.setLED(VisionLEDMode.kOn);
        } else if (!on) {
            limelight.setLED(VisionLEDMode.kDefault);
        } 
    }

    public boolean hasTarget() {
        // var result = limelight.getLatestResult();
        var result = arduCam.getLatestResult();

        return result.hasTargets();
    }

    public double getPitch() {
        // var result = limelight.getLatestResult();
        var result = arduCam.getLatestResult();

        if (result.hasTargets()) {
            return result.getBestTarget().getPitch();
        }
        return 0;
    }

    public double getSkew() {
        // only used for 3d targeting
        // var result = limelight.getLatestResult();
        var result = arduCam.getLatestResult();

        if (result.hasTargets()) {
            return result.getBestTarget().getSkew();
        }
        return 0;
    }

    public void setPiplineIndex(int index) {
        // limelight.setPipelineIndex(index);
        arduCam.setPipelineIndex(index);
    }

    public void setCameraDriverMode(boolean on) {
        arduCam.setDriverMode(on);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("camera yaw", getYaw());
        SmartDashboard.putNumber("camera pitch", getPitch());
        SmartDashboard.putNumber("camera skew", getSkew());
    }
}
