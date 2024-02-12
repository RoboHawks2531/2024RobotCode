package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shoot;

public class RevShooter extends Command{
    
    private Shoot shoot;
    private double rpm;

    public RevShooter(Shoot shoot, double rpm) {
        this.shoot = shoot;
        this.rpm = rpm;
    }

    @Override
    public void execute() {
        shoot.setMotorVelocity(rpm, false);
    }
}
