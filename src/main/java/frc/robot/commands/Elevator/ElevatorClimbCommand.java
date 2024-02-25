package frc.robot.commands.Elevator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class ElevatorClimbCommand extends Command{
    
    private Elevator elevator;

    public ElevatorClimbCommand(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // new ElevatorSetpointCommand(elevator, Constants.ElevatorConstants.lowSetpoint);
    }

    @Override
    public void execute() {
        double power = -MathUtil.applyDeadband(RobotContainer.operator.getLeftY(), Constants.stickDeadband);
        double lastExtension;
        double maxExtension = Constants.ElevatorConstants.highSetpoint; //encoder ticks
        double minExtension = Constants.ElevatorConstants.lowSetpoint; //encoder ticks
        double currentExtension = elevator.getEncoderLeft();

        if (power > 0 && currentExtension < maxExtension) {
            elevator.setMotors(power * Constants.ElevatorConstants.manualSpeed, power * Constants.ElevatorConstants.manualSpeed);
            lastExtension = elevator.getEncoderLeft();
        } else if (power < 0 && currentExtension > minExtension) {
            elevator.setMotors(power * Constants.ElevatorConstants.manualSpeed, power * Constants.ElevatorConstants.manualSpeed);
            lastExtension = elevator.getEncoderLeft();
        } else {
            elevator.setMotors(0, 0);
            // new ElevatorSetpointCommand(elevator, lastExtension);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setMotors(0, 0);
    }
}
