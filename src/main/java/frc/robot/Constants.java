//Updates 1/23/24 Maddie,Sam,Kaden
// Tuned Speeds, IDs, Recalibrated all Falcons Firmwear



package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(28); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(28); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 35;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        // public static final double openLoopRamp = 0.175; //turn back to a higher number if we make it to playoffs
        public static final double openLoopRamp = 0.25; 
        public static final double closedLoopRamp = 0.05;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 6.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-170.1563);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-152.4902325);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(27.5097675);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-66.1816475);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(113.8183525);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-89.560546875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class DeviceConstants {
        public static final int leftShooterMotor = 33; //only one shooter motor
        public static final int leftElevatorMotor = 28; //flip this, default is clockwise
        public static final int rightElevatorMotor = 13; //spinning counter clockwise
        public static final int intakePivotMotor = 31;
        public static final int intakePowerMotor = 21;
        public static final int indexMotorID = 34;
        public static final int shootPivotMotor = 22;
    }

    public static class ShootingConstants {
        public static final int targetShootingRPM = 6800;
        // public static final int targetShootingRPM = 3000;
        public static final int targetShootingAmpTarget = 1500;
        public static final double indexFeedVolts = 12;
        public static final double indexHoldVolts = 6;
        // public static final double pivotStore = 0.27;
        public static final double pivotStore = 0;
        public static final double pivotAmp = 64; //OUT IS NEGATIVE DO NOT PUT ANY POSITIVES OR IT BREAKS
        public static final double pivotIntake = 100;
        public static final double pivotDistanceShooting = 9;

    }

    public static class ElevatorConstants {
        public static final double manualSpeed = 0.70;
        public static final double manualDebugSpeed = 0.1;
        public static final double highSetpoint = 160; 
        public static final double midSetpoint = 25; 
        public static final double lowSetpoint = 2; 
    }

    public static class IntakeConstants {
        // public static final double sourceSetpoint = -33;
        // public static final double groundSetpoint = -125;
        // public static final double ampSetpoint = -44;
        public static final double sourceSetpoint = -33;
        // public static final double groundSetpoint = -122.5;
        public static final double groundSetpoint = -122;
        public static final double ampSetpoint = -20;

        // public static final double indexFeedingSetpoint = -3.5;
        // public static final double indexFeedingSetpoint = -2.95;
        // public static final double indexFeedingSetpoint = -1.2;
        public static final double indexFeedingSetpoint = -0.1;

        public static final double intakeSpitVolts = 5;
        public static final double intakeSuckVolts = -4;
    }

    public static class VisionConstants {
    // public static final String LimelightName = "2531limelight";
    
    /** Physical location of the camera on the robot, relative to the center of the robot. */
    public static final Transform3d robotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(6), 0, Units.inchesToMeters(18.75)), //TODO: This needs to be tuned to where the camera will be mounted 
        new Rotation3d(degreesToRadians(0), degreesToRadians(21.1), 0.0));
    
    public static final double FieldLengthMeters= 16.54175;
    public static final double FieldWidthMeters = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FlippingPose = new Pose2d(
        new Translation2d(FieldLengthMeters, FieldWidthMeters),
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double ApriltagAmbiguityThreshold = 0.2;

    public static final int ApriltagPiplineIndex = 0;
    public static final int IntakePiplineIndex = 1;
  }

    public static class MiscConstants {
        public static final int CANdleID = 20;
    }
}
