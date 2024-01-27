package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    private PhotonCamera limelight = new PhotonCamera("2531limelight");

    double cameraHeight = Units.inchesToMeters(0); //how hight is camera off the ground?
    double targetHeight = Units.inchesToMeters(0); //how high is the target off the ground(all april tags are the same)?
    double cameraPitchRadians = Units.degreesToRadians(0); //what is the cameras angle from level?
    double goalRangeMeters = Units.feetToMeters(3); //target goal distance for getting in range of a target

    public Vision() {

    }

    public double getDistanceMethod() {
        var result = limelight.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight, targetHeight, cameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return 0;
    }

    public double getYaw() {
        var result = limelight.getLatestResult();

        if (result.hasTargets()) {
            result.getBestTarget().getYaw();
        }
        return 0;
    }

    // public Transform2d translationMethod() {
    //     var result = limelight.getLatestResult();
    //     var target = result.getBestTarget();
    //     Transform2d pose = target.getCameraToTarget();

    //     if (result.hasTargets()) {
    //         return  result.getBestTarget().getBestCameraToTarget();
    //     }
    //     return new Transform2d();
    // }

}
