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

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** changes the color of the strip of lights it has */
public final class Lights extends SubsystemBase {
  public final AddressableLED lights = new AddressableLED(LightsConstants.kPort);
  public final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LightsConstants.kLength);
  public LEDPattern pattern = LEDPattern.solid(LightsConstants.kDefaultBlue);
  public final int numberOfLights = buffer.getLength();

  public Lights() {
    lights.setColorOrder(AddressableLED.ColorOrder.kRGB);
    lights.setSyncTime(LightsConstants.kMicrosecondsSync);
    lights.setLength(numberOfLights);
    lights.start();
  }

  @Override
  public void periodic() {
    // pattern.applyTo(buffer);
    lights.setData(buffer);
  }

  public final void gradientPattern(Color... colors) {
    pattern = LEDPattern.gradient(GradientType.kContinuous, colors);
  }
  
  /**
   * changes the pattern to a gradient with all the colors you pass as arguments.
   *
   * @param colors any number of Color arguments to go along the gradient
   * @return Command that makes it a gradient with all the colors
   */
  public final Command gradientPatternCommand(Color... colors) {
    return this.runOnce(()->gradientPattern(colors));
  }

  /**
   * Method to set the pattern to a solid color
   *
   * @param color The color to set the lights to
   */
  public void solidPattern(Color color) {
    pattern = LEDPattern.solid(color);
  }

  /**
   * changes the pattern to a solid color
   *
   * @param color which color to do
   * @return Command which changes it to a solid color
   */
  public final Command solidPatternCommand(Color color) {
    return this.runOnce(() -> solidPattern(color));
  }

  /** Method to set the pattern to rainbow */
  public void rainbowPattern() {
    pattern =
        LEDPattern.rainbow(LightsConstants.kBrightestColor, LightsConstants.kBrightestColor)
            .scrollAtAbsoluteSpeed(LightsConstants.kScrollSpeed, LightsConstants.kLEDSpacing);
  }

  /**
   * makes the pattern rainbow
   *
   * @return Command that does that
   */
  public final Command rainbowPatternCommand() {
    return this.runOnce(() -> rainbowPattern());
  }

  /**
   * Creates a new version of the current pattern at a different brightness.
   *
   * @param multiplier between 0 and 100
   * @return command that does it
   */
  public final void changeBrightness(byte multiplier) {
    if (multiplier < 0) {
      multiplier = 0;
    } else if (multiplier > 100) {
      multiplier = 100;
    }
    pattern.atBrightness(Percent.of(multiplier));
  }
  
  /**
   * changes the brightness but as a command
   * @param mulitplier between 0 and 100
   */
  public final Command changeBrightnessCommand(byte multiplier) {
    return this.runOnce(()->changeBrightness(multiplier));
  }

  /**
   * show and hide the current pattern instantly at an interval
   * 
   * @param time seconds to show it for
   * @param smooth whether its like a smooth pattern
   */
  public final void blink(double time, boolean smooth) {
    if (time > 10) {
      time = 10;
    } else if (time < 0) {
      time = 0;
    }
    if (smooth) {
      pattern = pattern.breathe(Seconds.of(time));
    } else {
      pattern = pattern.blink(Seconds.of(time));
    }
  }

  /**
   * @see blink()
   */
  public final Command blinkCommand(double time, boolean smooth) {
    return this.runOnce(()->blink(time, smooth));
  }

  public void setPattern(Color color, boolean rainbow) {
    if (rainbow) {
      rainbowPattern();
    } else {
      solidPattern(color);
    }
  }

  public final Command setPatternCommand(Supplier<Color> color, BooleanSupplier rainbow) {
    return run(() -> setPattern(color.get(), rainbow.getAsBoolean()));
  }

  public final Command blink(short milliseconds) {
    return this.runOnce(() -> {});
  }
}
