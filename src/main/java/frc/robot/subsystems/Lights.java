package frc.robot.subsystems;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** changes the color of the strip of lights it has */
public final class Lights extends SubsystemBase {
  public final AddressableLED lights = new AddressableLED(LightsConstants.kPort);
  public final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LightsConstants.kLength);
  public LEDPattern pattern = LEDPattern.solid(LightsConstants.kDefaultBlue);
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
   * changes the pattern to a gradient with all the colors you pass as arguments.
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
   * Method to set the pattern to a solid color
   *
   * @param color The color to set the lights to
   */
  public void setSolidPattern(Color color) {
    pattern = LEDPattern.solid(color);
  }

  /**
   * changes the pattern to a solid color
   *
   * @param color which color to do
   * @return Command which changes it to a solid color
   */
  public final Command solidPattern(Color color) {
    return this.runOnce(() -> setSolidPattern(color));
  }

  /** Method to set the pattern to rainbow */
  public void setRainbowPattern() {
    pattern =
        LEDPattern.rainbow(LightsConstants.kBrightestColor, LightsConstants.kBrightestColor)
            .scrollAtAbsoluteSpeed(LightsConstants.kScrollSpeed, LightsConstants.kLEDSpacing);
  }

  /**
   * makes the pattern rainbow
   *
   * @return COmmand that does that
   */
  public final Command rainbowPattern() {
    return this.runOnce(() -> setRainbowPattern());
  }

  /**
   * Creates a new version of the current pattern at a different brightness.
   *
   * @param multiplier between like 0.1 and 2
   * @return command that does it
   */
  public final Command multiplyBrightness(float multiplier) {
    return this.runOnce(
        () -> {
          // doesnt work
          pattern.atBrightness(Dimensionless.ofBaseUnits(multiplier, null));
        });
  }

  public void setPattern(Color color, boolean rainbow) {
    if (rainbow) {
      setRainbowPattern();
    } else {
      setSolidPattern(color);
    }
  }

  public final Command setPatternCommand(Supplier<Color> color, BooleanSupplier rainbow) {
    return run(() -> setPattern(color.get(), rainbow.getAsBoolean()));
  }

  public final Command blink(short milliseconds) {
    return this.runOnce(() -> {});
  }
}
