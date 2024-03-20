package frc.robot.commands.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.math.Conversions;
import frc.robot.subsystems.Intake;

public class IntakeSetpointCommand extends Command{
    private Intake intake;
    private double setpoint;

    /*
     * From what I could gather running simulations of PIDs on the WPILIB docs, The Integral Values can remain at 0,
     * while adjusting the P values to allow the system to reach the setpoint without oscillation, then adjusting the D value CAREFULLY 
     * to allow the system to reach the setpoint as quickly as possible without overshooting.
     * 
     * Basically alot more tuning is needed for a faster and more percise intake.
     * so... TODO: Tune intake PID to be constistantly fast and percise going to all setpoint values
     */
    //Old PID Values
    // private PIDController pidController = new PIDController(0.42, 0.015, 0);

    //New PID Values
    // private PIDController pidController = new PIDController(0.75, 0.0, 0.0);
    private PIDController pidController = new PIDController(0.95, 0.0, 0.0001);

    public IntakeSetpointCommand(Intake intake, double setpoint) {
        this.intake = intake;
        this.setpoint = setpoint;

        // pidController.setSetpoint(setpoint);
        pidController.setTolerance(0);
        // pidController.setIZone(20);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(intake.getPivotEncoder(), setpoint);

        intake.setPivotVolts(speed);

        SmartDashboard.putNumber("Intake PID Error", pidController.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSpeed(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
