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
  private AddressableLEDBuffer ledBufferCone;
  private AddressableLEDBuffer ledBufferCube;
  private AddressableLEDBuffer ledBufferGameObject;
  private AddressableLEDBuffer ledBufferNoTag;
  private AddressableLEDBuffer ledBufferWorking;
  private AddressableLEDBuffer ledBufferReadyToScore;
  private AddressableLEDBuffer ledBufferOff;

  /** for example, a spacing value of 1 would mean every LED is on */
  private int spacing = 1;

  /** timer for blinking LED */
  private int blinkingTimer = 0;
  /** When the blinkingTimer reaches 10 (equal to blinkingPeriod), the LED is toggled */
  private int blinkingPeriod = 10;
  /** If we are currently blinking, then True would mean the color is currently showing */
  private boolean blinkingColorOn = false;

  public Lights() {
    ledBufferOff = new AddressableLEDBuffer(Constants.LED_LENGTH);
    led.setLength(ledBufferOff.getLength());

    // sets the rgb values for each of the AddressableLEDBuffers
    for (int i = 0; i < ledBufferCone.getLength(); i = i + spacing) {
      ledBufferCone.setRGB(i, 255, 255, 0);
    }

    for (int i = 0; i < ledBufferCube.getLength(); i = i + spacing) {
      ledBufferCube.setRGB(i, 255, 0, 255);
    }

    for (int i = 0; i < ledBufferGameObject.getLength(); i = i + spacing) {
      ledBufferCube.setRGB(i, 0, 255, 0);
    }

    for (int i = 0; i < ledBufferNoTag.getLength(); i = i + spacing) {
      ledBufferNoTag.setRGB(i, 0, 255, 0);
    }

    for (int i = 0; i < ledBufferWorking.getLength(); i = i + spacing) {
      ledBufferWorking.setRGB(i, 255, 0, 0);
    }

    for (int i = 0; i < ledBufferReadyToScore.getLength(); i = i + spacing) {
      ledBufferReadyToScore.setRGB(i, 0, 0, 255);
    }

    for (int i = 0; i < ledBufferOff.getLength(); i = i + spacing) {
      ledBufferOff.setRGB(i, 0, 0, 0);
    }

    lightOptionsMap = new TreeMap<LightCode, AddressableLEDBuffer>();
    lightOptionsMap.put(LightCode.CONE, ledBufferCone);
    lightOptionsMap.put(LightCode.CUBE, ledBufferCube);
    lightOptionsMap.put(LightCode.GAME_OBJECT, ledBufferGameObject);
    lightOptionsMap.put(LightCode.NO_TAG, ledBufferNoTag);
    lightOptionsMap.put(LightCode.WORKING, ledBufferWorking);
    lightOptionsMap.put(LightCode.READY_TO_SCORE, ledBufferReadyToScore);
    lightOptionsMap.put(LightCode.OFF, ledBufferOff);
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
          led.setData(ledBufferOff);
        } else {
          led.setData(ledBufferNoTag);
        }
        blinkingColorOn = !blinkingColorOn;
      }
      blinkingTimer++;
    }
  }
}
