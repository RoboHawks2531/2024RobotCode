package frc.robot.subsystems;

import java.util.concurrent.Flow;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Candle extends SubsystemBase{

    private static final CANdle candle = new CANdle(Constants.MiscConstants.CANdleID);

    public static final Color black = new Color(0, 0, 0);
    public static final Color yellow = new Color(242, 60, 0);
    public static final Color purple = new Color(184, 0, 185);
    public static final Color white = new Color(255, 230, 220);
    public static final Color green = new Color(56, 209, 0);
    public static final Color blue = new Color(8, 32, 255);
    public static final Color red = new Color(255, 0, 0);
    public static final Color orange = new Color(227, 110, 7);
    

    public Candle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.stripType = LEDStripType.GRB;
        config.v5Enabled = true;
        config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        config.brightnessScalar = 1;
        candle.configAllSettings(config, 100);
        candle.configLEDType(LEDStripType.GRB); //just added this after cd post

        setDefaultCommand(defaultCommand());

        System.out.println("Candle Initialized");
    }

    public Command defaultCommand() {
        return runOnce(() -> {
            LEDSegment.MainStrip.fullClear();
            LEDSegment.InternalLEDs.fullClear();

            LEDSegment.MainStrip.setColor(purple);
            LEDSegment.InternalLEDs.setColor(purple);
            // LEDSegment.MainStrip.setFireAnimation(0.5, 0.25, 0.25);
        });
    }

    public static enum LEDSegment {
        InternalLEDs(0, 8, 0),
        MainStrip(8, 60, 1);
        //start index is what LED to start on, 0-7 are the candles onboard LEDS, beyond is the strip

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            candle.animate(animation, animationSlot);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            candle.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(black);
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 3, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }

        public void setFireAnimation(double speed, double cooling, double sparking) {
            setAnimation(new FireAnimation(1, speed, segmentSize, cooling, sparking));

        }

        public void setDefaultLEDColors() {
        }
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
        
    }
}











// @Deprecated
/* voided bins 
 * public void configLEDType(LEDStripType ledType) {
        candle.configLEDType(ledType);
    }

    public void setLEDColorRGB(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public void setLEDColorRGBW(int r, int g, int b, int w, int startIndex, int length) {
        candle.setLEDs(r, g, b, w, startIndex, length);
    }

    public void setLEDPurple() {
        candle.setLEDs(80, 45, 127);
        configBrighness(1);
    }

    public void setLEDGreen() {
        candle.setLEDs(0, 255, 0);
        configBrighness(1);
    }

    public void setLEDBlue() {
        candle.setLEDs(0, 0, 255);
        configBrighness(1);
    }

    public void setLEDRed() {
        candle.setLEDs(255, 0, 0);
        configBrighness(1);
    }

    public void configBrighness(int brightness) {
        candle.configBrightnessScalar(brightness);
    }

    public void setLEDOff() {
        candle.setLEDs(0, 0, 0);
    }

    public void setLEDRainbow() {
        RainbowAnimation rainbow = new RainbowAnimation();
        candle.animate(rainbow);
    }

    public void setLEDAnimation(Animation animation) {
        //Animation types: Rainbow, Larson, Fire, , Strobe, Twinkle On/Off, RGB Fade, Single Fade.
        candle.animate(animation);
    }

    public void setLEDLarson(int r, int g, int b, BounceMode mode) {
        LarsonAnimation larson = new LarsonAnimation(r,g,b);
        larson.setBounceMode(mode);
        candle.animate(larson);
    }

    public void setLEDStrobe(int r, int g, int b) {
        StrobeAnimation strobe = new StrobeAnimation(r,g,b);
        strobe.setSpeed(250);
        candle.animate(strobe);
    }

    public void setLEDFire(int brightness, int sparking, int cooling, int speed, int numLEDs) {
        FireAnimation fire = new FireAnimation(brightness, speed, numLEDs, sparking, cooling);
        candle.animate(fire);
    }

    public void setLEDTwinkleOn(int r, int g, int b) {
        TwinkleAnimation twinkle = new TwinkleAnimation(r,g,b);
        twinkle.setSpeed(250);
        candle.animate(twinkle);
    }

    public void setLEDTwinkleOff(int r, int g, int b) {
        TwinkleOffAnimation twinkle = new TwinkleOffAnimation(r,g,b);
        twinkle.setSpeed(250);
        candle.animate(twinkle);
    }

    public void setLEDRGBFade(int r, int g, int b) {
        RgbFadeAnimation fade = new RgbFadeAnimation(r,g,b);
        fade.setSpeed(250);
        candle.animate(fade);
    }

*/
