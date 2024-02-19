package frc.robot.commands.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.math.Conversions;
import frc.robot.subsystems.Intake;

public class IntakeSetpointCommand extends Command{
    private Intake intake;
    private double setpoint;
    // private PIDController pidController = new PIDController(0.40, 0.015, 0);
    private PIDController pidController = new PIDController(0.43, 0.015, 0);
    // private ArmFeedforward feedforward = new ArmFeedforward(0.1, 0.1, 0.1);

    public IntakeSetpointCommand(Intake intake, double setpoint) {
        this.intake = intake;
        this.setpoint = setpoint;

        // pidController.setSetpoint(setpoint);
        pidController.setTolerance(0);
        pidController.setIZone(20);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(intake.getPivotEncoder(), setpoint);
        // double ffSpeed = feedforward.calculate(setpoint, 2, 2);

        // intake.setPivotSpeed(speed);
        intake.setPivotVolts(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSpeed(0);
        // double reverseSpeed = pidController.calculate(intake.getPivotEncoder(), 0);
        // intake.setPivotSpeed(reverseSpeed);
    }
    
    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
