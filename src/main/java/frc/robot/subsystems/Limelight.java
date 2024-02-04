//We can use this if we switch back to the limelight software and discover that it runs better overall

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// This is a special subsystem. Do NOT use requirements on it!
public class Limelight extends SubsystemBase {
  // Time value used internally to represent that the light will never be ready without further calls,
  // probably because the lights are disabled.
  private static final long NEVER = Long.MAX_VALUE;

  // https://docs.limelightvision.io/en/latest/networktables_api.html
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry ledMode = table.getEntry("ledMode");

  private double subsystemUsedBy = 0;

  private long lightsWillBeReadyAtMillis = NEVER;

  public Limelight() {

  }

  public boolean hasValidTarget() {
    return tv.getDouble(0) == 1;
  }

  public double getX() {
    return tx.getDouble(0);
  }

  public double getY() {
    return ty.getDouble(0);
  }

  public double getArea() {
    return ta.getDouble(0);
  }

  public double getDistance() {
    double targetHeight = 53.88; // inches
    double mountHeight = 4; // inches
    double mountAngle = 50; // degrees
    double y = getY();
    return (targetHeight - mountHeight) / Math.tan((mountAngle + y) * (Math.PI / 180.0));
  }

  public void ensureEnabled() {
    subsystemUsedBy++;
    setLightsEnabled(subsystemUsedBy > 0);
  }

  public void noLongerNeeded() {
    subsystemUsedBy--;
    setLightsEnabled(subsystemUsedBy > 0);
  }

  public boolean isReady() {
    // We've observed that it takes a little bit of time for the lights to actually turn on and start tracking.
    // Commands should wait until the light is ready before using numbers. Commands can assume that once the
    // limelight has become ready, it will always stay ready for the duration of the command.
    return System.currentTimeMillis() >= lightsWillBeReadyAtMillis;
  }

  private void setLightsEnabled(boolean enabled) {
    // 0 - pipeline default
    // 1 - force off
    // 2 - force blink
    // 3 - force on
    int newMode = enabled ? 3 : 0;
    if (newMode != ledMode.getNumber(0).intValue()) {
      System.out.println("Limelight enabled: " + enabled);
      if (enabled) {
        lightsWillBeReadyAtMillis = System.currentTimeMillis() + 250;
      } else {
        lightsWillBeReadyAtMillis = NEVER;
      }
      ledMode.setNumber(newMode);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight Distance", hasValidTarget() ? getDistance() : -1);
  }
}
