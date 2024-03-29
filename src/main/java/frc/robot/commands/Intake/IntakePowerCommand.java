package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakePowerCommand extends Command{
    private Intake intake;
    private double volts;
    
    public IntakePowerCommand(Intake intake, double volts) {
        this.intake = intake;
        this.volts = volts;

        // addRequirements(intake); //uncomment this if its being overruled by other commands
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.setPowerVolts(volts);
    }

    @Override
    public void end(boolean interrupted) {
        // if (volts > 0) {
        //     intake.setPowerVolts(0.2);
        // } else {
        //     intake.setPowerVolts(-0.2);
        // }
        // intake.setPivotVolts(volts / 2);
        intake.setPowerVolts(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}