package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;

public class ResetShooter extends Command{
    
    private Intake intake;
    private Shoot shoot;

    public ResetShooter(Intake intake, Shoot shoot) {
        this.intake = intake;
        this.shoot = shoot;
        
        addRequirements(intake, shoot);
    }

    @Override
    public void execute() {
        // intake.setIntakeAndSetpoint(0, 0);
        intake.setPivotSpeed(0); //readd this if something goes wrong
        intake.setPowerVolts(0);
        shoot.setIndexMotorVolts(0);
        shoot.setPivotMotorSpeed(0);
        shoot.setSplitMotorVolts(0, 0);
        shoot.coastMotors();
        new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
