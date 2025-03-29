package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

/** changes the color of the strip of lights it has */
public final class Lights extends SubsystemBase {
  public final AddressableLED lights = new AddressableLED(LightsConstants.kPort);
  public final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LightsConstants.kLength);
  public LEDPattern pattern =
      LEDPattern.rainbow(LightsConstants.kBrightestColor, LightsConstants.kBrightestColor);
  public final short numberOfLights = (short) buffer.getLength();

  /** when it's going between 2 colors, which it should go to */
  public boolean alternateFirstColor = true;

  public Lights() {
    lights.setColorOrder(AddressableLED.ColorOrder.kRGB);
    lights.setSyncTime(LightsConstants.kMicrosecondsSync);
    lights.setLength(numberOfLights);
    lights.start();
  }

  public void periodic() {
    pattern.applyTo(buffer);
    lights.setData(buffer);
  }

  /**
   * changes the pattern to a gradient with all the colors you pass as arguments. Dont too too many
   * arguments or else thats stupid
   *
   * @param colors
   * @return Command that makes it a gradient with all the colors
   */
  public final Command gradientPattern(Color... colors) {
    return this.runOnce(
        () -> {
          pattern = LEDPattern.gradient(GradientType.kContinuous, colors);
        });
  }

  /**
   * changes the pattern to a solid color
   *
   * @param color which color to do
   * @return Command which changes it to a solid color
   */
  public final Command solidPattern(Color color) {
    return this.runOnce(
        () -> {
          pattern = LEDPattern.solid(color);
        });
  }

  /**
   * makes the pattern rainbow
   *
   * @return COmmand that does that
   */
  public final Command rainbowPattern() {
    return this.runOnce(
        () -> {
          pattern =
              LEDPattern.rainbow(LightsConstants.kBrightestColor, LightsConstants.kBrightestColor);
        });
  }

  /**
   * multiplies the brightness to be brighter or darker, 1 does nothing It doenst work yet cause
   * they use some "dimensionless" thing instead of a double like ANYBODY SANE WOULD DO!!!! AHHHH!!
   * im just kidding im not really mad its just stupid
   *
   * @param multiplier between like 0.1 and 2 i dont really know
   * @return command that does it
   */
  public final Command multiplyBrightness(float multiplier) {
    return this.runOnce(()->{
      //doesnt work
      pattern.atBrightness(Dimensionless.ofBaseUnits(multiplier, null));
    });
  }

  public final Command blink(short milliseconds) {
    return this.runOnce(()->{

    });
  }
}
