package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import java.util.TreeMap;

/** Controls the LEDs that indicate robot status */
public class Lights extends SubsystemBase {
  private LightCode currentLightStatus = LightCode.OFF;
  private TreeMap<LightCode, AddressableLEDBuffer> lightOptionsMap;
  private AddressableLED led = new AddressableLED(RobotMap.LED_PWM_PORT);

  /** for example, a spacing value of 1 would mean every LED is on */
  private int spacing = 1;

  /** timer for blinking LED */
  private int blinkingTimer = 0;
  /** When the blinkingTimer reaches 10 (equal to blinkingPeriod), the LED is toggled */
  private int blinkingPeriod = 10;
  /** If we are currently blinking, then True would mean the color is currently showing */
  private boolean blinkingColorOn = false;

  public enum LightCode {
    CONE, // Solid Yellow
    CUBE, // Solid Purple
    GAME_OBJECT, // Solid Green
    NO_TAG, // Blinking Green
    WORKING, // Solid Red
    READY_TO_SCORE, // Blue
    OFF
  }

  public Lights() {
    lightOptionsMap = new TreeMap<LightCode, AddressableLEDBuffer>();
    lightOptionsMap.put(LightCode.CONE, new AddressableLEDBuffer(Constants.LED_LENGTH));
    lightOptionsMap.put(LightCode.CUBE, new AddressableLEDBuffer(Constants.LED_LENGTH));
    lightOptionsMap.put(LightCode.GAME_OBJECT, new AddressableLEDBuffer(Constants.LED_LENGTH));
    lightOptionsMap.put(LightCode.NO_TAG, new AddressableLEDBuffer(Constants.LED_LENGTH));
    lightOptionsMap.put(LightCode.WORKING, new AddressableLEDBuffer(Constants.LED_LENGTH));
    lightOptionsMap.put(LightCode.READY_TO_SCORE, new AddressableLEDBuffer(Constants.LED_LENGTH));
    lightOptionsMap.put(LightCode.OFF, new AddressableLEDBuffer(Constants.LED_LENGTH));

    led.setLength(lightOptionsMap.get(LightCode.OFF).getLength());

    setUpBuffer(lightOptionsMap.get(LightCode.CONE), 255, 255, 0);
    setUpBuffer(lightOptionsMap.get(LightCode.CUBE), 255, 0, 255);
    setUpBuffer(lightOptionsMap.get(LightCode.GAME_OBJECT), 0, 255, 0);
    setUpBuffer(lightOptionsMap.get(LightCode.NO_TAG), 0, 255, 0);
    setUpBuffer(lightOptionsMap.get(LightCode.WORKING), 255, 0, 0);
    setUpBuffer(lightOptionsMap.get(LightCode.READY_TO_SCORE), 0, 0, 255);
    setUpBuffer(lightOptionsMap.get(LightCode.OFF), 0, 0, 0);
  }

  private void setUpBuffer(AddressableLEDBuffer buffer, int r, int g, int b) {
    for (var i = 0; i < buffer.getLength(); i += spacing) {
      buffer.setRGB(i, 255, 0, 0);
    }
  }

  public void toggleCode(LightCode light) {
    if (currentLightStatus == light) {
      currentLightStatus = LightCode.OFF;
    } else {
      currentLightStatus = light;
    }
    led.setData(lightOptionsMap.get(currentLightStatus));
    led.start();
  }

  public void setLight(LightCode light) {
    currentLightStatus = light;
    led.setData(lightOptionsMap.get(currentLightStatus));
    led.start();
  }

  /** Sets the spacing between each LED so not all are lit up when applicable */
  public void setSpacing(int s) {
    spacing = s;
  }

  @Override
  public void periodic() {
    /** NO_TAG is the only LightCode that requires blinking */
    if (currentLightStatus == LightCode.NO_TAG) {
      if (blinkingTimer >= blinkingPeriod) {
        if (blinkingColorOn) {
          led.setData(lightOptionsMap.get(LightCode.OFF));
        } else {
          led.setData(lightOptionsMap.get(LightCode.NO_TAG));
        }
        blinkingColorOn = !blinkingColorOn;
      }
      blinkingTimer++;
    }
  }
}
