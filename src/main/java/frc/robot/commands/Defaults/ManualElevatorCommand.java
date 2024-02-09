package frc.robot.commands.Defaults;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command{
    
    private Elevator elevator;
    private DoubleSupplier Supplier;

    public ManualElevatorCommand(Elevator elevator, DoubleSupplier Supplier) {
        this.elevator = elevator;
        this.Supplier = Supplier;

        addRequirements(elevator);
    }


    @Override
    public void execute() {
        double speed = MathUtil.applyDeadband(Supplier.getAsDouble(), Constants.stickDeadband);
   
        elevator.setMotors(speed * Constants.ElevatorConstants.manualSpeed, speed * Constants.ElevatorConstants.manualSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setMotors(0, 0);
    }
}
