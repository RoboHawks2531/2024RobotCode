// 1/23/24 Maddie, Sam, Kaden 
//Changed Joystick to XboxController

// 1/24/24 Kaden
//added shoot subsystem and constant shooting using voltage

// 1/26/24 Kaden
//added velocity shooting controls
//TODO: get vision subsystem framework and a sample rotation command

// 1/27/24 Kaden
//added a vision rotation command and buttons to activate

package frc.robot;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);

    private PhotonCamera camera = new PhotonCamera("2531Limelight");

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); //map to button 7 for two squares
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton brakeMotors = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Shooting Controls */
    private final JoystickButton shootVolts = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton shootVelocitySlow = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton shootVelocity = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);

    /* Vision Controls */
    private final JoystickButton rotateToTarget = new JoystickButton(driver, 7); //maps to the two squares on controller

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();
    private final Shoot shoot = new Shoot();

    // private Supplier<Pose2d> poseSupplier;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();


        autoChooser.addOption("Example Auto", new exampleAuto(s_Swerve));

        autoChooser.addOption("Sequential Testing Auto", new SequentialTestingAuto(s_Swerve));

        autoChooser.addOption("Aim And Shoot Auto", new AimAndShoot(s_Swerve, vision, shoot));

        SmartDashboard.putData(autoChooser);
    }


    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /* Debug Buttons */
        brakeMotors.onTrue(new InstantCommand(() -> shoot.brakeMotors()));
        brakeMotors.onFalse(new InstantCommand(() -> shoot.coastMotors()));

        /* Shooting Commands */
        //using velocity vs. voltage helps with shooting at a constant, rather than it deviating when battery is over/under charged
        shootVelocity.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, false)));
        shootVelocitySlow.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, true)));
        shootVolts.onTrue(new InstantCommand(() -> shoot.setSplitMotorVolts(-10,-10)));
        shootVolts.onFalse(new InstantCommand(() -> shoot.setMotorVolts(0)));

        /* Vision Commands */
        rotateToTarget.whileTrue(new RotateToTarget(s_Swerve, vision));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
