package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command{
    
    private Elevator elevator;

    public ManualElevatorCommand(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }


    @Override
    public void execute() {
        double speed = -MathUtil.applyDeadband(RobotContainer.operator.getLeftY() , Constants.stickDeadband);
   
        elevator.setMotors(speed * Constants.ElevatorConstants.manualSpeed, speed * Constants.ElevatorConstants.manualSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setMotors(0, 0);
        elevator.zeroMotorEncoders();
    }
}
