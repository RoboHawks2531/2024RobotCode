// 1/28/24 Kaden
// added this in case we need to use it instead of the motion magic stuff


package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorSetpointCommand extends Command{
    
    private Elevator elevator;
    private PIDController pidController1 = new PIDController(0.12, 0.012, 0);
    private PIDController pidController2 = new PIDController(0.12, 0.012, 0);
    
    private double setpoint;
    

    //this is here incase we want to tell it to move to a certain distance using meters instead of ticks
    private final double circumference = 0.5 * Math.PI; // meters
    private final double gearRatio = 75.0 / 1.0; // output / input
    private final double encoderResolution = 2048.0; // ticks / output
    private final double tick2Meters = setpoint * (circumference / (gearRatio * encoderResolution));
    private final double meters2Tick = setpoint * (gearRatio * encoderResolution / circumference);


    public ElevatorSetpointCommand(Elevator elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        // addRequirements(elevator);
        pidController1.setIZone(20);
        pidController2.setIZone(20);
        pidController1.setSetpoint(setpoint);
        pidController2.setSetpoint(setpoint);
    }

    @Override
    public void initialize() {
        //womp womp
        // pidController1.reset();
        // pidController2.reset();
    }

    @Override
    public void execute() {
        double speed1 = pidController1.calculate(elevator.getEncoder1());
        double speed2 = pidController2.calculate(elevator.getEncoder2());

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
