// 1/28/24 Kaden
// added this in case we need to use it instead of the motion magic stuff


package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorSetpointCommand extends Command{
    
    private Elevator elevator;
    private PIDController pidController1 = new PIDController(0.005, 0.0, 0);
    private PIDController pidController2 = new PIDController(0.005, 0.0, 0);

    private double setpoint;

    public ElevatorSetpointCommand(Elevator elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        // addRequirements(elevator);
        pidController1.setIZone(30);
        pidController2.setIZone(30);
        pidController1.setSetpoint(setpoint);
        pidController2.setSetpoint(setpoint);
    }

    @Override
    public void initialize() {
        //womp womp
        pidController1.reset();
        pidController2.reset();
    }

    @Override
    public void execute() {
        double speed1 = pidController1.calculate(elevator.getEncoderLeft());
        double speed2 = pidController2.calculate(elevator.getEncoderRight());

        elevator.setMotors(speed1, speed2);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return pidController1.atSetpoint() && pidController2.atSetpoint();
    }
}
