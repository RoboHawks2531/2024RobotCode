package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command{

    Elevator elevator;
    double speed;
    
    public MoveElevator(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
    }

    @Override
    public void execute() {
        elevator.setMotors(speed, speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
