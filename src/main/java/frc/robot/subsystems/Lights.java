package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

/** changes the color of the strip of lights it has */
public final class Lights extends SubsystemBase {
  public final AddressableLED lights = new AddressableLED(LightsConstants.kPort);
  public final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LightsConstants.kLength);
  public final int numberOfLights = buffer.getLength();
  /**when it's going between 2 colors, which it should go to*/
  public boolean alternateFirstColor = true;

  public Lights() {
    lights.start();
    lights.setColorOrder(AddressableLED.ColorOrder.kRGB);
    lights.setSyncTime(0);
  }

  /**
   * changes red, green, and blue values from Color values (between 0 and 1) to int values between 0
   * and 255
   *
   * @param color from Color value, between 0 and 1
   * @return int between 0 and 255
   */
  public final int colorToInt(double color) {
    if (color > 1) {
        color = 1;
        //TODO print a warning
    } else if (color < 0) {
        color = 0;
        //TODO print another warning
    }
    return (int) color * 255;
  }

  /**
   * change color with wpilib Color value
   *
   * @param i which light to change, must be between 0 and the last light
   * @param color Color value to change
   */
  public final void changeOneColor(int i, Color color) {
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
  public final void changeOneColor(int i, int red, int green, int blue) {
    if (i > numberOfLights || i < 0) {
        //probably messed up the for loop and its going twice
        //TODO print yet ANOther warning
        return;
    }
    int[] allTheLights = {red, green, blue};
    for (int thing : allTheLights) {
      if (thing > LightsConstants.kBrightestColor) {
        thing = LightsConstants.kBrightestColor;
        //TODO so many warnigng I CANT TAKE IT anymore
      } else if (thing < 0) {
        thing = 0;
        //TODO crashout
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
  public final void solidColor(Color color) {
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
  public final void solidColor(int red, int green, int blue) {
    for (int i = 0; i < numberOfLights; i++) {
      changeOneColor(i, red, green, blue);
    }
  }

  /**
   * converts red, green, and blue to an int array of hue, saturation value colors without the weird
   * unpacking stuff TODO make this into an object with red, green, blue properties instead of an
   * int array cause theyre werid to work with
   *
   * @return
   */
  public final int[] hsvToRgb(int red, int green, int blue) {
    int[] colors = new int[3];
    int packedColor = Color.hsvToRgb(red, green, blue);
    colors[0] = Color.unpackRGB(packedColor, Color.RGBChannel.kRed);
    colors[1] = Color.unpackRGB(packedColor, Color.RGBChannel.kGreen);
    // shoutout kasane teto
    // and the blue one... I GUESS...
    colors[2] = Color.unpackRGB(packedColor, Color.RGBChannel.kBlue);
    return colors;
  }

  /**
   * makes a static rainbow pattern with brightest and most saturated hsv
   *
   * <p>Idk how to make it move around and do all that fancy stuff
   */
  public final void rainbow() {
    for (int i = 0; i < numberOfLights; i++) {
      int[] colors =
          hsvToRgb(
              (int) (i / numberOfLights * LightsConstants.kMaxHue),
              LightsConstants.kBrightestColor,
              LightsConstants.kBrightestColor);
      changeOneColor(i, colors[0], colors[1], colors[2]);
    }
  }

  /**
   * goes between 2 solid colors
   * Doesnt work yet
   * @param firstColor the first color to go to
   * @param secondColor the other color to go to
   */
  public final void alternate(Color firstColor, Color secondColor) {
    if (alternateFirstColor) {
        solidColor(firstColor);
    } else {
        solidColor(secondColor);
    }
  }

  /** makes it all blu e i guess */
  public final void solidBlue() {
    solidColor(Color.kBlue);
  }
}
