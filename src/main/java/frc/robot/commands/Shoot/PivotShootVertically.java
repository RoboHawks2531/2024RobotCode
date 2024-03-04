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

    @Override
    public void execute() {
        // double verticalOffset = 10;
        double targetHeight = Units.feetToMeters(6);

        if (vision.hasTarget()) {
        //   new PivotPIDCommand(shoot, Math.tan(targetHeight / vision.getDistanceMethod()));  
          new PivotPIDCommandNonDegrees(shoot, -Conversions.degreesToFalcon(Math.atan(targetHeight / vision.getDistanceMethod()), .008)); //this needs to be adjusted to the actual gear ratio of the pivot
        //   new PivotPIDCommandNonDegrees(shoot, -Math.atan(targetHeight / vision.getDistanceMethod())); //this needs to be adjusted to the actual gear ratio of the pivot
        } else {
            new PivotPIDCommand(shoot, Constants.ShootingConstants.pivotStore); //brings the pivot to a default position if no target is found
        }
        
    }
}
