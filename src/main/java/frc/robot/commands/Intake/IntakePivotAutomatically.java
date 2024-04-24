package frc.robot.commands.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Vision;

public class IntakePivotAutomatically extends Command{

    private Intake intake;
    private Vision vision;
    private PIDController pidController = new PIDController(1.3, 0.0, 0.0001);

    public IntakePivotAutomatically(Intake intake, Vision vision) {
        this.intake = intake;
        this.vision = vision;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        pidController.reset();
        pidController.setTolerance(0);
    }

    //this will most likely never be used, but it is here just in case
    @Override
    public void execute() {
        double speakerHeight = 70;
        double shooterHeight = 30;
        // double angularOffset = -41.5;
        double angularOffset = -15;

        // double angle = Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / Units.metersToInches(vision.getDistanceMethod())));
        double angle = Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / Units.metersToInches(2.0)));
        double angModulus = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angle + angularOffset)));

        pidController.setSetpoint(MathUtil.clamp(-angModulus, 0, -20));
        // pidController.setSetpoint(angModulus);
        double setpoint = pidController.getSetpoint();

        double speed = pidController.calculate(intake.getPivotEncoder(), setpoint);
        intake.setPivotSpeed(-speed);

        System.out.println("intake :" + setpoint + "  speed" + speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotSpeed(0);
    }
}
