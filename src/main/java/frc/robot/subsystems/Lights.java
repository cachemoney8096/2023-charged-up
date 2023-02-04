package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Controls the LEDs that indicate robot status */
public class Lights extends SubsystemBase {
  private LightCode currentLightStatus = LightCode.OFF;
  private AddressableLED led = new AddressableLED(RobotMap.LED_PWM_PORT);
  private AddressableLEDBuffer ledBuffer;
  /** for example, a spacing value of 1 would mean every LED is on */
  private int spacing = 1;
  /** timer for blinking led */
  private int blinkingTimer = 0;

  public Lights() {
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());
  }

  public enum LightCode {
    CONE, // Solid Yellow
    CUBE, // Solid Purple
    GAME_OBJECT, // Solid Green
    NO_TAG, // Blinking Green
    WORKING, // Solid Red
    READY_TO_SCORE, // Blue
    OFF
  }

  public void toggleCode(LightCode light) {
    if (currentLightStatus == light) {
      currentLightStatus = LightCode.OFF;
    } else {
      currentLightStatus = light;
    }
    setColor(currentLightStatus);
    led.setData(ledBuffer);
    led.start();
  }

  public void setLight(LightCode light) {
    currentLightStatus = light;
    setColor(currentLightStatus);
    led.setData(ledBuffer);
    led.start();
  }

  /** Sets the spacing between each LED so not all are lit up when applicable */
  public void setSpacing(int s) {
    spacing = s;
  }

  /** sets the color of the LEDs in RGB */
  public void setColor(LightCode light) {
    int r = 0;
    int g = 0;
    int b = 0;
    switch (light) {
      case CONE:
        r = 255;
        g = 255;
        b = 0;
        break;
      case CUBE:
        r = 255;
        g = 0;
        b = 255;
        break;
      case GAME_OBJECT:
        r = 0;
        g = 255;
        b = 0;
        break;
      case NO_TAG:
        r = 0;
        g = 255;
        b = 0;
        break;
      case WORKING:
        r = 255;
        g = 0;
        b = 0;
        break;
      case READY_TO_SCORE:
        r = 0;
        g = 0;
        b = 255;
        break;
      case OFF:
        r = 0;
        g = 0;
        b = 0;
        break;
      default:
        // this should never trigger
        r = 0;
        g = 0;
        b = 0;
    }

    for (int i = 0; i < ledBuffer.getLength(); i = i + spacing) {
      ledBuffer.setRGB(i, r, g, b);
    }
  }

  @Override
  public void periodic() {
    if (currentLightStatus == LightCode.NO_TAG) {
      if (blinkingTimer == 500) {
        toggleCode(LightCode.NO_TAG);
        blinkingTimer = 0;
      }
    }
    blinkingTimer += 50;
  }
}
