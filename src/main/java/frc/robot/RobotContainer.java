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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.Defaults.TeleopSwerve;
import frc.robot.commands.Intake.IntakePowerCommand;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Shoot.AimAndShoot;
import frc.robot.commands.Vision.RotateToTarget;
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

    /* Driver button usage ($ means used)
     * A$,X$,Y$,B$, Left Bumper$, Right Bumper$, Left Trigger$, Right Trigger$, Menu, Two Squares$
     */

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); //map to button 7 for two squares
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); //map to button __ for menu

    /* Intake Buttons */
    private final JoystickButton intakeSource = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intakeGround = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intakeStore = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton intakeSuck = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intakeSpit = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    // private final JoystickButton brakeMotors = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Elevator Buttons */


    /* Shooting Controls */
    private final JoystickButton shootVolts = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton shootVelocitySlow = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton shootNoneAim = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    // private final JoystickButton shootVelocity = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
    private final JoystickButton shootAiming = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);

    /* Vision Controls */
    private final JoystickButton rotateToTarget = new JoystickButton(driver, 7); //this will be changes to zero gyro

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();
    private final Shoot shoot = new Shoot();
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();

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

        autoChooser.addOption("Aim And Shoot Auto", new AimAndShoot(s_Swerve, vision, shoot, intake));

        autoChooser.addOption("red Alliance test", new RedAllianceTestAuto(s_Swerve, vision));

        autoChooser.addOption("Sams Very Long Auto", new TwoRingAutoCauseWeReallyThinkWeCanDoThisByWeekOne(s_Swerve, intake, shoot, vision));

        SmartDashboard.putData(autoChooser);

        intake.zeroPivotEncoder();
        elevator.zeroMotorEncoders();
    }


    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /* Debug Buttons */
        // brakeMotors.onTrue(new InstantCommand(() -> shoot.brakeMotors()));
        // brakeMotors.onFalse(new InstantCommand(() -> shoot.coastMotors()));

        /* Shooting Commands */
        //using velocity vs. voltage helps with shooting at a constant, rather than it deviating when battery is over/under charged
        // shootVelocity.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, false)));
        
        shootNoneAim.whileTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, false)));
        shootAiming.whileTrue(new AimAndShoot(s_Swerve, vision, shoot, intake));
        // shootVelocitySlow.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, true)));

        shootVolts.onTrue(new InstantCommand(() -> shoot.setSplitMotorVolts(-10,-10)));
        shootVolts.onFalse(new InstantCommand(() -> shoot.setMotorVolts(0)));

        /* Intake Commands */
        intakeStore.onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, 0),
            new IntakePowerCommand(intake, 2)
        ));

        intakeGround.onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, -25),
            new IntakePowerCommand(intake, 3)
        ));

        intakeSource.onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, -10),
            new IntakePowerCommand(intake, 3)
        ));

        intakeSuck.whileTrue(new InstantCommand(() -> intake.setPivotSpeed(-0.2)));
        intakeSpit.whileTrue(new InstantCommand(() -> intake.setPivotSpeed(0.2)));
        intakeSpit.whileFalse(new InstantCommand(() -> intake.setPivotSpeed(0)));
        intakeSuck.whileFalse(new InstantCommand(() -> intake.setPivotSpeed(0)));

        //uncomment these when we remove the debug shooting commands
        // intakeSuck.whileTrue(new IntakePowerCommand(intake, 3));
        // intakeSpit.whileTrue(new IntakePowerCommand(intake, -3));

        /* Elevator Commands */


        /* Vision Commands */
        rotateToTarget.whileTrue(new RotateToTarget(s_Swerve, vision));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
