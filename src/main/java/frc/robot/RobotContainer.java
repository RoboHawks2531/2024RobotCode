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
import frc.robot.commands.Defaults.TeleopSwerve;
// import frc.robot.commands.Elevator.ElevatorClimbCommand;
import frc.robot.commands.Elevator.ElevatorSetpointCommand;
// import frc.robot.commands.Elevator.MoveElevator;
// import frc.robot.commands.Elevator.ManualElevatorCommand;
import frc.robot.commands.Intake.IntakePowerCommand;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Intake.ManualPivotIntake;
import frc.robot.commands.Shoot.AimAndShoot;
import frc.robot.commands.Shoot.AmpShoot;
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
    public static final CommandXboxController driver = new CommandXboxController(0);
    // public static final CommandXboxController operator = new CommandXboxController(1);

    // private PhotonCamera camera = new PhotonCamera("2531Limelight");

    /* Driver button usage ($ means used)
     * A$,X$,Y$,B$, Left Bumper$, Right Bumper$, Left Trigger$, Right Trigger$, Menu, Two Squares$
     */

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();
    private final Shoot shoot = new Shoot();
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // s_Swerve.setDefaultCommand(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> -driver.getRawAxis(translationAxis), 
        //         () -> -driver.getRawAxis(strafeAxis), 
        //         () -> -driver.getRawAxis(rotationAxis), 
        //         () -> driver.start().getAsBoolean()
        //     )
        // );
        
        shoot.setDefaultCommand(
            new PivotShootVertically(shoot, vision)
        );

        
        autoChooser.addOption("Example Auto", new exampleAuto(s_Swerve));

        autoChooser.addOption("Sequential Testing Auto", new SequentialTestingAuto(s_Swerve));

        // autoChooser.addOption("Aim And Shoot Auto", new AimAndShoot(s_Swerve, vision, shoot, intake));

        // autoChooser.addOption("Red Alliance Auto", new RedAllianceTestAuto(s_Swerve, vision));

        SmartDashboard.putData(autoChooser);

        intake.zeroPivotEncoder();
        elevator.zeroMotorEncoders(); 
        shoot.zeroPivotEncoder();
        
        // Configure the button bindings
        configureButtonBindings();
    }


    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driver.start().onTrue(new InstantCommand(() -> intake.zeroPivotEncoder()));

        /* Shooting Commands */
        //using velocity vs. voltage helps with shooting at a constant, rather than it deviating when battery is over/under charged
        // driver.rightTrigger(0.3).whileTrue(new AimAndShoot(s_Swerve, vision, shoot, intake));
        // driver.leftTrigger(0.3).whileTrue(new AuxShoot(intake, shoot));

        // driver.rightTrigger(0.3).whileFalse(new ResetShooter(intake, shoot));
        // driver.leftTrigger(0.3).whileFalse(new ResetShooter(intake, shoot));

        // shootVelocity.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, false)));
        
        // shootNoneAim.whileTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, false)));
        // shootAiming.whileTrue(new AimAndShoot(s_Swerve, vision, shoot, intake));
        // // shootVelocitySlow.onTrue(new InstantCommand(() -> shoot.setMotorVelocity(5, true)));

        // shootVolts.onTrue(new InstantCommand(() -> shoot.setSplitMotorVolts(-10,-10)));
        // shootVolts.onFalse(new InstantCommand(() -> shoot.setMotorVolts(0)));

        // driver.back().whileTrue(new InstantCommand(() -> shoot.setSplitMotorVolts(3, 3)));
        // driver.back().whileFalse(new InstantCommand(() -> shoot.setSplitMotorVolts(0, 0)));
        driver.back().whileFalse(new ResetShooter(intake, shoot));
        driver.back().whileTrue(new AuxShoot(intake, shoot));

        /* Intake Commands */
        driver.x().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, 0)
            // new IntakePowerCommand(intake, 2)
        ));

        driver.a().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.groundSetpoint)
            // new IntakePowerCommand(intake, 3)
        ));

        driver.b().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.sourceSetpoint)
            // new IntakePowerCommand(intake, 3)
        ));

        driver.y().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.ampSetpoint)
            // new IntakePowerCommand(intake, rotationAxis) //it dont need this tbh
        ));

        /* Intake Manual Pivoting */
        driver.rightTrigger(0.5).whileTrue(new ManualPivotIntake(intake, 0.15)); // Intake Pivot Up
        driver.leftTrigger(0.5).whileTrue(new ManualPivotIntake(intake, -0.15)); // Intake Pivot Down
        
        /* Intake Power */
        // driver.rightBumper().whileTrue(new IntakePowerCommand(intake, 4)); // left bumper
        // driver.leftBumper().whileTrue(new IntakePowerCommand(intake, -3)); // right bumper
        driver.leftBumper().whileTrue(new InstantCommand(() -> intake.setPowerVelocity(Constants.IntakeConstants.intakeSuckVelocity, false)));
        driver.leftBumper().whileTrue(new InstantCommand(() -> intake.setPowerVelocity(Constants.IntakeConstants.intakeSpitVelocity, false)));

        driver.povLeft().whileTrue(new ParallelCommandGroup(
            // new InstantCommand(() -> shoot.setIndexMotorVolts(6)),
            new InstantCommand(() -> shoot.setIndexMotorVelocity(Constants.ShootingConstants.indexFeedVelocity)),
            new InstantCommand(() -> shoot.setMotorVelocity(-Constants.ShootingConstants.targetShootingAmpTarget, false))
        ));

        driver.povRight().whileTrue(new AmpShoot(shoot, intake));

        driver.povLeft().whileFalse(new ResetShooter(intake, shoot));
        driver.povRight().whileFalse(new ResetShooter(intake, shoot));

        /* Elevator Commands */
        // operator.a().onTrue(new ElevatorSetpointCommand(elevator, 0));
        // operator.b().onTrue(new ElevatorSetpointCommand(elevator, 25));
        // operator.y().onTrue(new ElevatorSetpointCommand(elevator, 50));

        // operator.back().toggleOnTrue(new ManualElevatorCommand(elevator));
        // operator.start().toggleOnTrue(new ElevatorClimbCommand(elevator));

        /* Vision Commands */
        // rotateToTarget.whileTrue(new RotateToTarget(s_Swerve, vision));
        // driver.povUp().whileTrue(new MoveElevator(elevator, 0.2));
        // driver.povDown().whileTrue(new MoveElevator(elevator, -0.2));
        
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
