package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ManualPivotIntake extends Command{
    
    private Intake intake;
    private double speed;

    public ManualPivotIntake(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;

        addRequirements(intake);
    }   

    @Override
    public void execute() {
        intake.setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
