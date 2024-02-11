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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.Defaults.ManualElevatorCommand;
import frc.robot.commands.Defaults.TeleopSwerve;
import frc.robot.commands.Elevator.ElevatorSetpointCommand;
import frc.robot.commands.Intake.IntakePowerCommand;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Intake.ManualPivotIntake;
import frc.robot.commands.Shoot.AimAndShoot;
import frc.robot.commands.Shoot.AuxShoot;
import frc.robot.commands.Shoot.PivotShootVertically;
import frc.robot.commands.Shoot.ResetShooter;
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
    private final CommandXboxController driver = new CommandXboxController(0);
    private final XboxController operator = new XboxController(1);

    // private PhotonCamera camera = new PhotonCamera("2531Limelight");

    /* Driver button usage ($ means used)
     * A$,X$,Y$,B$, Left Bumper$, Right Bumper$, Left Trigger$, Right Trigger$, Menu, Two Squares$
     */

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value); //map to button 7 for two squares
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value); //map to button __ for menu

    // /* Intake Buttons */
    // private final JoystickButton intakeSource = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton intakeGround = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton intakeStore = new JoystickButton(driver, XboxController.Button.kX.value);

    // private final JoystickButton intakeSuck = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton intakeSpit = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    // // private final JoystickButton intakePivotUp = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
    // // private final JoystickButton intakePivotDown = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);


    // private final POVButton intakePivotUp = new POVButton(driver, 0);
    // private final POVButton intakePivotDown = new POVButton(driver, 180);

    // private final JoystickButton brakeMotors = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Elevator Buttons */
    private final JoystickButton elevatorHighSetpoint = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton elevatorMidSetpoint = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton elevatorStoreSetpoint = new JoystickButton(operator, XboxController.Button.kA.value);

    /* Shooting Controls */
    // private final JoystickButton shootVolts = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton shootVelocitySlow = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    // private final JoystickButton shootNoneAim = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    // private final JoystickButton shootVelocity = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
    // private final JoystickButton shootAiming = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);

    /* Vision Controls */
    // private final JoystickButton rotateToTarget = new JoystickButton(driver, 7); //this will be changed to zero gyro

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
                () -> driver.start().getAsBoolean()
            )
        );
        
        elevator.setDefaultCommand(
            new ManualElevatorCommand(
                elevator,
                () -> -operator.getLeftY()
            )
        );

        shoot.setDefaultCommand(
            new PivotShootVertically(shoot, vision)
        );

        
        autoChooser.addOption("Example Auto", new exampleAuto(s_Swerve));

        autoChooser.addOption("Sequential Testing Auto", new SequentialTestingAuto(s_Swerve));

        autoChooser.addOption("Aim And Shoot Auto", new AimAndShoot(s_Swerve, vision, shoot, intake));

        autoChooser.addOption("Red Alliance Auto", new RedAllianceTestAuto(s_Swerve, vision));

        SmartDashboard.putData(autoChooser);

        intake.zeroPivotEncoder();
        elevator.zeroMotorEncoders(); 
        
        // Configure the button bindings
        configureButtonBindings();
    }


    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /* Shooting Commands */
        //using velocity vs. voltage helps with shooting at a constant, rather than it deviating when battery is over/under charged
        driver.rightTrigger(0.3).whileTrue(new AimAndShoot(s_Swerve, vision, shoot, intake));
        driver.leftTrigger(0.3).whileTrue(new AuxShoot(intake, shoot));

        driver.rightTrigger(0.3).whileFalse(new ResetShooter(intake, shoot));
        driver.leftTrigger(0.3).whileFalse(new ResetShooter(intake, shoot));

        // shootVelocity.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, false)));
        
        // shootNoneAim.whileTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, false)));
        // shootAiming.whileTrue(new AimAndShoot(s_Swerve, vision, shoot, intake));
        // // shootVelocitySlow.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, true)));

        // shootVolts.onTrue(new InstantCommand(() -> shoot.setSplitMotorVolts(-10,-10)));
        // shootVolts.onFalse(new InstantCommand(() -> shoot.setMotorVolts(0)));

        /* Intake Commands */
        driver.x().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, 0)
            // new IntakePowerCommand(intake, 2)
        ));

        driver.a().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, -100),
            new IntakePowerCommand(intake, 1.5)
        ));

        driver.b().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, -55)
            // new IntakePowerCommand(intake, 3)
        ));

        /* Intake Manual Pivoting */
        driver.rightTrigger(0.5).whileTrue(new ManualPivotIntake(intake, 0.2)); // Intake Pivot Up
        driver.leftTrigger(0.5).whileTrue(new ManualPivotIntake(intake, -0.2)); // Intake Pivot Down
        
        /* Intake Power */
        driver.rightBumper().whileTrue(new IntakePowerCommand(intake, 2)); // left bumper
        driver.leftBumper().whileTrue(new IntakePowerCommand(intake, -2)); // right bumper

        /* Elevator Commands */
        elevatorStoreSetpoint.onTrue(new ElevatorSetpointCommand(elevator, 0)); // kA
        elevatorMidSetpoint.onTrue(new ElevatorSetpointCommand(elevator, 25)); // kX
        elevatorHighSetpoint.onTrue(new ElevatorSetpointCommand(elevator, 50)); //kY

        /* Vision Commands */
        // rotateToTarget.whileTrue(new RotateToTarget(s_Swerve, vision));
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
