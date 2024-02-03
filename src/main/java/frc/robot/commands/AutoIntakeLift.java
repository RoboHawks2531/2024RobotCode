package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeLift extends Command {
  private final Intake intake;
  private final double setpoint;

  public AutoIntakeLift(Intake intake, double setpoint) {
    this.intake = intake;
    this.setpoint = setpoint;
    
    addRequirements(intake);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    boolean hasNote = intake.getLimitSwitch();

    if (hasNote) {
        intake.setIntakeAndSetpoint(-0.2, 0);
    } else if (!hasNote) {
        intake.setIntakeAndSetpoint(0.5, setpoint);
    } 


  }

  @Override
  public void end(boolean interrupted) {
    intake.setPowerVolts(-2);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
