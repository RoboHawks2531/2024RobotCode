package frc.robot.commands.Defaults;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command{
    
    private Elevator elevator;
    private DoubleSupplier upSupplier;
    private DoubleSupplier downSupplier;

    public ManualElevatorCommand(Elevator elevator, DoubleSupplier upSupplier, DoubleSupplier downSupplier) {
        this.elevator = elevator;
        this.upSupplier = upSupplier;
        this.downSupplier = downSupplier;

        addRequirements(elevator);
    }


    @Override
    public void execute() {
        double upSpeed = MathUtil.applyDeadband(upSupplier.getAsDouble(), Constants.stickDeadband);
        double downSpeed = MathUtil.applyDeadband(downSupplier.getAsDouble(), Constants.stickDeadband);

        if (upSpeed > 0) {
            elevator.setMotors(upSpeed * Constants.ElevatorConstants.manualSpeed, upSpeed * Constants.ElevatorConstants.manualSpeed);
        } else if (downSpeed > 0) {
            elevator.setMotors(downSpeed * Constants.ElevatorConstants.manualSpeed, downSpeed * Constants.ElevatorConstants.manualSpeed);
        } else if (upSpeed > 0 && downSpeed > 0) {
            elevator.setMotors(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setMotors(0, 0);
    }
}
