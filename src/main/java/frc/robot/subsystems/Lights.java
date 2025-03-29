package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
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

  /**
   * changes a red, green, or blue value from Color values (between 0 and 1) to int values between 0
   * and 255
   *
   * @param color from Color value, between 0 and 1
   * @return int between 0 and 255
   */
  private final short colorToInt(double color) {
    if (color > 1) {
      color = 1;
      // TODO print a warning
    } else if (color < 0) {
      color = 0;
      // TODO print another warning
    }
    Integer realColor = (int) color * 255;
    return realColor.shortValue();
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
  /*public final Command multiplyBrightness(double multiplier) {
    return this.runOnce(()->{
      //TODO this probably doesnt work
      pattern.atBrightness(Dimensionless.ofBaseUnits(multiplier, null));
    });
  }

  public final Command blink(byte seconds) {
    return this.runOnce(()->{
    });
  }

  /**
   * change color with wpilib Color value
   *
   * @param i which light to change, must be between 0 and the last light
   * @param color Color value to change
   */
  /*public final void changeOneColor(short i, Color color) {
    changeOneColor(i, colorToInt(color.red), colorToInt(color.green), colorToInt(color.blue));
  }

  /**
   * change color to RGB, basically the same as {@link AddressableLEDBuffer.setRGB}
   *
   * <p>Its recommended you use the one with the Color argument so you dont have magic numbers of
   * red green and blue
   *
   * @param i which color, must be less than buffer.length
   * @param red between 0 and 255
   * @param green between 0 and 255
   * @param blue between 0 and 255
   */
  /*public final void changeOneColor(short i, short red, short green, short blue) {
    if (i > numberOfLights || i < 0) {
      // probably messed up the for loop and its going twice
      // TODO print yet ANOther warning
      return;
    }
    int[] allTheLights = {red, green, blue};
    for (int thing : allTheLights) {
      if (thing > LightsConstants.kBrightestColor) {
        thing = LightsConstants.kBrightestColor;
        // TODO so many warnigng I CANT TAKE IT anymore
      } else if (thing < 0) {
        thing = 0;
        // TODO crashout
      }
    }
    buffer.setRGB(i, red, green, blue);
    lights.setData(buffer);
  }

  /**
   * changed everything to the same color instantly
   *
   * @param color the Color value to change to
   */
  /*public final void solidColor(Color color) {
    solidColor(colorToInt(color.red), colorToInt(color.green), colorToInt(color.blue));
  }

  /**
   * Changes everything to one color instantly, its recommended you use the other one with the Color
   * value
   *
   * @param red red, between 0 and 255
   * @param green green, between 0 and 255
   * @param blue blue, between 0 and 255
   */
  /*public final void solidColor(short red, short green, short blue) {
    for (short i = 0; i < numberOfLights; i++) {
      changeOneColor(i, red, green, blue);
    }
  }

  /**
   * converts hue, saturation, and value into an IntColors with red, green, blue properties
   * @param hue between 0 and 180
   * @param saturation between 0 and 255
   * @param value between 0 and 255
   * @return IntColor
   */
  /*public final IntColors hsvToRgb(short hue, short saturation, short value) {
    while (hue < 0) {
        //goes around the hue circle basically
        hue += LightsConstants.kMaxHue;
    }
    hue %= LightsConstants.kMaxHue;
    if (saturation > LightsConstants.kBrightestColor) {
        saturation = LightsConstants.kBrightestColor;
    } else if (saturation < 0) {
        saturation = 0;
    }
    int packedColor = Color.hsvToRgb(hue, saturation, value);
    return new IntColors(
        (short) Color.unpackRGB(packedColor, Color.RGBChannel.kRed),
        (short) Color.unpackRGB(packedColor, Color.RGBChannel.kGreen),
        (short) Color.unpackRGB(packedColor, Color.RGBChannel.kBlue)
    );
  }

  /**
   * makes a static rainbow pattern with brightest and most saturated hsv
   *
   * <p>Idk how to make it move around and do all that fancy stuff
   */
  /*public final void rainbow() {
    pattern.applyTo(buffer);
    for (short i = 0; i < numberOfLights; i++) {
      IntColors colors =
          hsvToRgb(
              (short) (i / numberOfLights * LightsConstants.kMaxHue),
              LightsConstants.kBrightestColor,
              LightsConstants.kBrightestColor
              );
      changeOneColor(i, colors.red, colors.green, colors.blue);
    }
  }

  /**color but red, green, and blue are ints between 0 and 255 and not double 0 to 1*/
  public final class IntColors {
    public short red, green, blue;

    /**
     * makes a new class of IntColor
     *
     * @param red between 0 and 255
     * @param green between 0 and 255
     * @param blue between 0 and 255
     */
    public IntColors(short red, short green, short blue) {
      short[] args = {red, green, blue};
      for (int color : args) {
        if (color < 0) {
          color = 0;
        } else if (color > LightsConstants.kBrightestColor) {
          color = LightsConstants.kBrightestColor;
        }
      }
      this.red = args[0];
      this.green = args[1];
      this.blue = args[2];
    }
  }
}
