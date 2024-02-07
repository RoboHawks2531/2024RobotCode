package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeSetpointCommand extends Command{
    private Intake intake;
    private double setpoint;
    private PIDController pidController = new PIDController(0.025, 0.007, 0);
    // private PIDController pidController = new PIDController(0.015, 0.009, 0);

    public IntakeSetpointCommand(Intake intake, double setpoint) {
        this.intake = intake;
        this.setpoint = setpoint;

        // pidController.setSetpoint(setpoint);
        pidController.setTolerance(0);
    }

    @Override
    public void initialize() {
        // pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(intake.getPivotEncoder(), setpoint);

        intake.setPivotSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSpeed(0);
        // double reverseSpeed = pidController.calculate(intake.getPivotEncoder(), 0);
        // intake.setPivotSpeed(reverseSpeed);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
