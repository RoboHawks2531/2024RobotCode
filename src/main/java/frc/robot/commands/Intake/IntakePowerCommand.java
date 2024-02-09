package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePowerCommand extends Command{
    private Intake intake;
    private double volts;
    
    public IntakePowerCommand(Intake intake, double volts) {
        this.intake = intake;
        this.volts = volts;

        addRequirements(intake);
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
        //     intake.setPowerVolts(0.5);
        // } else {
        //     intake.setPowerVolts(-0.5);
        // }
        intake.setPowerVolts(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
