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


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Defaults.TeleopSwerve;
import frc.robot.commands.Elevator.ElevatorClimbCommand;
import frc.robot.commands.Elevator.ManualElevatorCommand;
// import frc.robot.commands.Elevator.ManualElevatorCommand;
import frc.robot.commands.Intake.IntakePowerCommand;
import frc.robot.commands.Intake.IntakeSetpointCommand;
import frc.robot.commands.Shoot.AimAndShoot;
import frc.robot.commands.Shoot.AmpShoot;
import frc.robot.commands.Shoot.AuxShoot;
import frc.robot.commands.Shoot.DistanceShoot;
import frc.robot.commands.Shoot.FeedingShoot;
import frc.robot.commands.Shoot.IndexNote;
import frc.robot.commands.Shoot.PivotPIDCommandNonDegrees;
import frc.robot.commands.Shoot.PivotShootVertically;
import frc.robot.commands.Shoot.PulseNote;
import frc.robot.commands.Shoot.ResetShooter;
import frc.robot.commands.Shoot.RevShooter;
import frc.robot.commands.Vision.RotateToTarget;
import frc.robot.commands.Vision.TagToPoseTrajectoryGenerator;
import frc.robot.commands.Vision.VisionTranslate;
// import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);

    // private PhotonCamera camera = new PhotonCamera("2531Limelight");

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
    // private final Candle candle = new Candle();

    // private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private SendableChooser<Command> autoChooser = new SendableChooser<>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                // () -> driver.leftTrigger(0.5).getAsBoolean()
                () -> driver.leftStick().getAsBoolean()
            )
        );
        

        NamedCommands.registerCommand("Aux Shoot", new AuxShoot(intake, shoot).withTimeout(1.7)); //full shooting command for auto
        NamedCommands.registerCommand("Intake Ground", new ParallelCommandGroup( //this makes sure the intake is down and ends when it needs to, if stopping early, switch to the deadline group version
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.groundSetpoint),
            new IntakePowerCommand(intake, -3)
        ).withTimeout(1.8));
        NamedCommands.registerCommand("Intake Ground No Timeout", new ParallelCommandGroup(  //put this in a deadline group with the path as the deadline to ensure that the intake picks up the note
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.groundSetpoint),
            new IntakePowerCommand(intake, -3)
        ));
        NamedCommands.registerCommand("Intake Ground Quick Timeout", new ParallelCommandGroup( //this is used to ensure that the intake is at the ground for intaking
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.groundSetpoint),
            new IntakePowerCommand(intake, -3)
        ).withTimeout(0.5));
        NamedCommands.registerCommand("Reset Shooter", new ResetShooter(intake, shoot)); //stops the shooter and index
        NamedCommands.registerCommand("Amp Shoot", new AmpShoot(shoot, intake).withTimeout(3)); //runs the amp shooting command; this involves the pivot which needs to be returned to store at the end
        NamedCommands.registerCommand("Intake Store", new IntakeSetpointCommand(intake, -2.5).withTimeout(1.5)); //bring the intake to the store position
        NamedCommands.registerCommand("Zero Swerve", new InstantCommand(() -> s_Swerve.zeroHeading())); // use this at the end of the routine so that we are zeroed for teleop
        NamedCommands.registerCommand("Zero All", new ParallelCommandGroup( //use only when we start facing the center of the field
            new InstantCommand(() -> s_Swerve.zeroHeading()),
            new InstantCommand(() -> intake.zeroPivotEncoder()),
            new InstantCommand(() -> shoot.zeroPivotEncoder())
        ));
        NamedCommands.registerCommand("Zero Intake", new InstantCommand(() -> shoot.zeroPivotEncoder()).withTimeout(0.1)); //use both at the beginning of each auto sequence
        NamedCommands.registerCommand("Zero Pivot", new InstantCommand(() -> intake.zeroPivotEncoder()).withTimeout(0.1)); //use both at the beginning of each auto sequence
        NamedCommands.registerCommand("Shooter Pivot Store", new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore).withTimeout(1)); //use with amp auto to zero
        NamedCommands.registerCommand("Pulse Stage 2", new ParallelCommandGroup( //brings the note from the index to the intake
            new InstantCommand(() -> shoot.setIndexMotorVolts(-3)),
            new IntakePowerCommand(intake, -3)).withTimeout(0.2));
        NamedCommands.registerCommand("Pulse Stage 1", new ParallelCommandGroup( //brings the note into the index from the intake
            new IntakeSetpointCommand(intake, -4),
            new InstantCommand(() -> shoot.setIndexMotorVolts(3)),
            new IntakePowerCommand(intake, 3)).withTimeout(0.2));
        NamedCommands.registerCommand("Aim And Shoot", new AimAndShoot(s_Swerve, vision, shoot, intake)); //aiming shooting command
        NamedCommands.registerCommand("Wait Command", new WaitCommand(7)); //waiting command for delaying auto a lot
        NamedCommands.registerCommand("Wait Command 1", new WaitCommand(1)); //using this to delay shooting to not hit any notes while shooting
        // NamedCommands.registerCommand("Vision Trajectory to Speaker", new TagToPoseTrajectoryGenerator(s_Swerve, vision, 7, 36));
        NamedCommands.registerCommand("Vision Translate", new VisionTranslate(s_Swerve, vision, 1.32, 0));
        NamedCommands.registerCommand("Spin Spin", new RunCommand(() -> s_Swerve.drive(new Translation2d(), 0.3, false, false)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        
        // autoChooser.addOption("Example Auto", new exampleAuto(s_Swerve));

        // autoChooser.addOption("Two Note Auto", new TestingTwoNoteAuto(s_Swerve, vision, shoot, intake));

        // SmartDashboard.putData(autoChooser);

        intake.zeroPivotEncoder(); //zeros intake on startup ONLY
        elevator.zeroMotorEncoders(); //zeros elevator on startup ONLY
        shoot.zeroPivotEncoder(); //zeros shooter pivot on startup ONLY

        // vision.getCamera().setPipelineIndex(0); //change to index of april tag detection
        // vision.getIntakeCamera().setPipelineIndex(1); //change to the index of the note detection

        SmartDashboard.putData("Zero Intake", new InstantCommand(() -> intake.zeroPivotEncoder())); //creates a dashboard button to zero intake
        SmartDashboard.putData("Zero Pivot", new InstantCommand(() -> shoot.zeroPivotEncoder())); //creates a dashboard button to zero shooter pivot
        SmartDashboard.putData("Zero Elevator", new InstantCommand(() -> elevator.zeroMotorEncoders())); //creates a dashboard button to zero elevator
        
        // Configure the button bindings
        configureButtonBindings();
    }


    private void configureButtonBindings() {
        /* Any command that uses the shooter or intake must be fitted with a while false statement to bring it to a stop */

        /* Driver Buttons */
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // driver.button(7).whileTrue(new TagToPoseTrajectoryGenerator(s_Swerve, vision, 3, 15));
        driver.button(7).whileTrue(new VisionTranslate(s_Swerve, vision, 1.32, 0));
        // driver.start().onTrue(new InstantCommand(() -> intake.zeroPivotEncoder()));
        // driver.start().onTrue(new InstantCommand(() -> shoot.zeroPivotEncoder()));

        /* Intake Commands */
        driver.x().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.indexFeedingSetpoint),
            new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore)
            // new IntakePowerCommand(intake, 2)
        ));

        driver.a().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.groundSetpoint),
            new IntakePowerCommand(intake, -3.5)
        ));

        driver.b().onTrue(new ParallelCommandGroup(
            new IntakeSetpointCommand(intake, Constants.IntakeConstants.sourceSetpoint),
            new IntakePowerCommand(intake, -3)
        ));

        // driver.y().onTrue(new ParallelCommandGroup(
        //     new IntakeSetpointCommand(intake, Constants.IntakeConstants.ampSetpoint)
        //     // new IntakePowerCommand(intake, -3) //it dont need this tbh
        // ));
        
        /* Intake Power */
        driver.leftBumper().whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> shoot.setIndexMotorVolts(-3)),
            new IntakePowerCommand(intake, -3)));
        driver.rightBumper().whileTrue(new ParallelCommandGroup(
            new IntakePowerCommand(intake, 4),
            new InstantCommand(() -> shoot.setIndexMotorVolts(3))
        ));

        driver.leftBumper().onFalse(new ResetShooter(intake, shoot));
        driver.rightBumper().onFalse(new ResetShooter(intake, shoot));

        /* Shooting Commands */
        driver.rightTrigger(0.5).whileTrue(new SequentialCommandGroup(
            // new PulseNote(intake, shoot).withTimeout(0.9),
            new AuxShoot(intake, shoot)
            )
        );
        driver.rightTrigger(0).whileFalse(new ResetShooter(intake, shoot));

        //this is the first stage of the amp shoot
        driver.leftTrigger(0.5).whileTrue(new SequentialCommandGroup(
            // new IndexNote(intake, shoot).withTimeout(0.4),
            new ParallelCommandGroup(
            new InstantCommand(() -> intake.setPowerVolts(4)),
            // new InstantCommand(() -> intake.setPowerVelocity(Constants.IntakeConstants.intakeSpitVelocity, false)),
            new InstantCommand(() -> shoot.setIndexMotorVolts(8)).withTimeout(1.2)),
            new InstantCommand(() -> shoot.setMotorVelocity(-1, false)), // TODO: remove this if it shreds a note
            // new InstantCommand(() -> intake.setPowerVolts(0)),
            new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotAmp)
            ));
        driver.leftTrigger(0).whileFalse(new ResetShooter(intake, shoot));
        // driver.leftTrigger(0).whileFalse(new InstantCommand(() -> shoot.coastMotors()));

        driver.y().whileTrue(new ParallelCommandGroup(
                new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotAmp),
                new InstantCommand(() -> shoot.setIndexMotorVolts(8)),
                new InstantCommand(() -> shoot.setMotorVelocity(Constants.ShootingConstants.targetShootingAmpTarget, false))
                // new RunCommand(() -> Candle.LEDSegment.MainStrip.setStrobeAnimation(candle.blue, 0.5))
                // new RunCommand(() -> Candle.LEDSegment.MainStrip.setRainbowAnimation(0.5))
        ));

        driver.y().whileFalse(new ResetShooter(intake, shoot));

        driver.povLeft().whileTrue(new PulseNote(intake, shoot));
        driver.povLeft().whileFalse(new ResetShooter(intake, shoot));

        driver.povRight().whileTrue(new DistanceShoot(intake, shoot, vision));

        driver.povRight().whileFalse(new ResetShooter(intake, shoot));
        // driver.povRight().whileTrue(new RotateToTarget(s_Swerve, vision));
        // driver.povRight().whileTrue(new VisionTranslate(s_Swerve, vision, 2, 0));

        // driver.povDown().whileTrue(new DistanceShoot(intake, shoot));
        // driver.povDown().whileTrue(new PivotShootVertically(shoot, vision));
        driver.povDown().whileFalse(new ResetShooter(intake, shoot));

        driver.povDown().onTrue(
            new DistanceShoot(intake, shoot, vision)

            // new ParallelCommandGroup(
            // new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotIntake),
            // // new InstantCommand(() -> shoot.setMotorVelocity(-500, false))
            // new InstantCommand(() -> shoot.setIndexMotorVolts(2))
            // )
        );

        driver.povUp().whileTrue(new FeedingShoot(shoot, intake, vision));
        driver.povUp().whileFalse(new ResetShooter(intake, shoot));
            
        // driver.povRight().whileFalse(new ResetShooter(intake, shoot));

        /* Elevator Commands */
        // operator.a().onTrue(new ElevatorSetpointCommand(elevator, 7));
        // operator.b().onTrue(new ElevatorSetpointCommand(elevator, 25));
        // operator.y().onTrue(new ElevatorSetpointCommand(elevator, 50));

        operator.back().toggleOnTrue(new ManualElevatorCommand(elevator));
        // operator.a().onTrue(new IntakeSetpointCommand(intake, Constants.IntakeConstants.groundSetpoint));
        operator.start().toggleOnTrue(new ElevatorClimbCommand(elevator));

        operator.a().onTrue(new PivotPIDCommandNonDegrees(shoot, 15));
        operator.x().onTrue(new PivotPIDCommandNonDegrees(shoot, 30));

        // operator.a().whileTrue(new TagToPoseTrajectoryGenerator(s_Swerve, vision, vision.getBestTargetID(), 24));

        /* Vision Commands */
        // rotateToTarget.whileTrue(new RotateToTarget(s_Swerve, vision));        
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        // This will run whatever is selected in the dropdown box on shuffleboard
        return autoChooser.getSelected();
    }
}
