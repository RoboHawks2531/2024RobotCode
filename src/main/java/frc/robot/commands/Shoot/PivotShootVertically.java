package frc.robot.commands.Shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Vision;

public class PivotShootVertically extends Command{

    private Shoot shoot;
    private Vision vision;
    private PIDController pidController = new PIDController(0.3, 0, 0);

    public PivotShootVertically(Shoot shoot, Vision vision) {
        this.shoot = shoot;
        this.vision = vision;

        addRequirements(shoot, vision);
    }

    @Override
    public void initialize() {
        pidController.reset();
        pidController.setTolerance(0.5);
    }

    //this will most likely never be used, but it is here just in case
    @Override
    public void execute() {
        double speakerHeight = 70;
        double shooterHeight = 30;
        double angularOffset = -41.2;
        // double angularOffset = -42.8;

        double angle = Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / Units.metersToInches(vision.getDistanceMethod())));
        //  double angle = Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / Units.metersToInches(2.3622)));
        double angModulus = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angle + angularOffset)));

        pidController.setSetpoint(MathUtil.clamp(angModulus, -30, -0.1));
        double setpoint = -pidController.getSetpoint();

        double speed = pidController.calculate(shoot.getPivotEncoder(), setpoint);
        shoot.setPivotMotorSpeed(speed);

        System.out.println("Shooter :" + setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        shoot.setPivotMotorSpeed(0);
    }
}
