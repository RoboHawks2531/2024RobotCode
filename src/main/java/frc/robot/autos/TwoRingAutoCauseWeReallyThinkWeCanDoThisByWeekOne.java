package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.AutoIntakeLift;
import frc.robot.commands.Shoot.AimAndShoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TwoRingAutoCauseWeReallyThinkWeCanDoThisByWeekOne extends SequentialCommandGroup{

    private Swerve swerve;
    private Intake intake;
    private Shoot shoot;
    private Vision vision;

    public TwoRingAutoCauseWeReallyThinkWeCanDoThisByWeekOne(Swerve swerve, Intake intake, Shoot shoot, Vision vision) {
        this.swerve = swerve;
        this.intake = intake;
        this.shoot = shoot;
        this.vision = vision;




        addCommands(
            new AimAndShoot(swerve, vision, shoot, intake),
            new AutoIntakeLift(intake, Constants.IntakeConstants.groundSetpoint),
            new RunCommand(() -> swerve.zeroHeading()).withTimeout(0.1),
            new ParallelRaceGroup(
                new InstantCommand(() -> swerve.drive(new Translation2d(0.15/ 20 , 0), 0, false, true), swerve),
                new InstantCommand(() -> swerve.drive(new Translation2d(0.15/ 20,0), 0, false, true), swerve).withTimeout(3)
                
                
                //limit swicth end on both true
            ), 
            new RunCommand(() -> swerve.zeroHeading()),
            //flip up intake and move ring
            new AimAndShoot(swerve, vision, shoot, intake)
        );
    }
}
