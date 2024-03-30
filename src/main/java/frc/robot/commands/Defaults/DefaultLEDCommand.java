package frc.robot.commands.Defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candle;

public class DefaultLEDCommand extends Command{
    private Candle candle;


    public DefaultLEDCommand(Candle candle) {
        this.candle = candle;
        this.addRequirements(candle);
    }

    @Override
    public void execute() {
        // candle.setLEDStrobe(0,0,0);
    }
}
