package frc.robot.commands.Shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Vision;

public class PivotShootVertically extends Command{

    private Shoot shoot;
    private Vision vision;

    public PivotShootVertically(Shoot shoot, Vision vision) {
        this.shoot = shoot;
        this.vision = vision;

        addRequirements(shoot);
    }

    //this will most likely never be used, but it is here just in case
    @Override
    public void execute() {
        // double verticalOffset = 10;
        // double targetHeight = Units.feetToMeters(6.65);
        //   new PivotPIDCommand(shoot, Math.tan(targetHeight / vision.getDistanceMethod()));  
        if (vision.hasTarget()) {
          new PivotPIDCommandNonDegrees(shoot, MathUtil.clamp(-vision.getDistanceMethod() * 4.7, -15, -0.5));
        } else {
            new PivotPIDCommandNonDegrees(shoot, Constants.ShootingConstants.pivotStore);
        }
    }
}
