// package frc.robot.commands.Elevator;


// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Elevator;

// public class ElevatorClimbCommand extends Command{
    
//     private Elevator elevator;

//     public ElevatorClimbCommand(Elevator elevator) {
//         this.elevator = elevator;

//         addRequirements(elevator);
//     }

//     @Override
//     public void initialize() {
        
//     }

//     @Override
//     public void execute() {
//         double power = -MathUtil.applyDeadband(RobotContainer.operator.getLeftY(), Constants.stickDeadband);

//         double maxExtension = 100; //encoder ticks
//         double minExtension = 5; //encoder ticks
//         double currentExtension = elevator.getEncoder1();

//         if (power > 0 && currentExtension < maxExtension) {
//             elevator.setMotors(power * Constants.ElevatorConstants.manualSpeed, power * Constants.ElevatorConstants.manualSpeed);
//         } else if (power < 0 && currentExtension > minExtension) {
//             elevator.setMotors(power * Constants.ElevatorConstants.manualSpeed, power * Constants.ElevatorConstants.manualSpeed);
//         } else {
//             elevator.setMotors(0, 0);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         elevator.setMotors(0, 0);
//     }
// }
