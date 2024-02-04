package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeSetpointCommand extends Command{
    private Intake intake;
    private double setpoint;
    private PIDController pidController = new PIDController(0.012, 0, 0);

    public IntakeSetpointCommand(Intake intake, double setpoint) {
        this.intake = intake;
        this.setpoint = setpoint;

        pidController.setSetpoint(setpoint);
        pidController.setTolerance(0);
    }

    @Override
    public void initialize() {
        // pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(intake.getPivotEncoder());

        intake.setPivotSpeed(speed);

        if (pidController.atSetpoint()) {

        }
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
