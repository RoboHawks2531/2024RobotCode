// 1/27/24 Kaden
// added getPitch and getYaw methods to aid vision aiming

// 1/31/24 Kaden
// Fixed limelight so that it actually works

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    private PhotonCamera limelight = new PhotonCamera("2531Limelight");

    double cameraHeight = Units.inchesToMeters(4); //how hight is camera off the ground?
    double targetHeight = Units.inchesToMeters(53.88); //how high is the target off the ground(all april tags are the same)?
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
        limelight.setLED(VisionLEDMode.kOn);
        var result = limelight.getLatestResult();

        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        
        if (result.hasTargets()) {
            // targets.add(result.getBestTarget());
            // return targets.get(0).getYaw(); //this will hopefully filter the multitargets and pick one to aim at
            return target.getYaw(); //This should be the one to take the better target and go to that.. actually this might do the same thing as before

            // return result.getBestTarget().getYaw();
        }
        return 0;
    }

    public void takeSnapshot() {
        var result = limelight.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        
        if (result.hasTargets()) {
            limelight.takeOutputSnapshot();
        }
    }

    public void setCameraLEDS(boolean on) {
        if (on) {
            limelight.setLED(VisionLEDMode.kOn);
        } else if (!on) {
            limelight.setLED(VisionLEDMode.kDefault);
        } 
    }

    public double getPitch() {
        var result = limelight.getLatestResult();

        if (result.hasTargets()) {
            return result.getBestTarget().getPitch();
        }
        return 0;
    }

    public double getSkew() {
        
        var result = limelight.getLatestResult();

        if (result.hasTargets()) {
            return result.getBestTarget().getSkew();
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
    @Override
    public void periodic() {
        SmartDashboard.putNumber("camera yaw", getYaw());
        SmartDashboard.putNumber("camera pitch", getPitch());
        // SmartDashboard.putNumber("camera skew", getSkew());
    }
}
