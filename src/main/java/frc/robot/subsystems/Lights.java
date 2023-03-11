package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.TreeMap;

/** Controls the LEDs that indicate robot status */
public class Lights extends SubsystemBase {
  private LightCode currentLightStatus = LightCode.OFF;
  private TreeMap<LightCode, Double> lightOptionsMap;
  private Spark m_blinkin = new Spark(RobotMap.LED_PWM_PORT);

  /** timer for blinking LED */
  private int blinkingTimer = 0;
  /** When the blinkingTimer reaches 10 (equal to blinkingPeriod), the LED is toggled */
  private int blinkingPeriod = 10;
  /** If we are currently blinking, then True would mean the color is currently showing */
  private boolean blinkingColorOn = false;

  /** true if party mode is currently on */
  private boolean partyMode = false;

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
    lightOptionsMap = new TreeMap<LightCode, Double>();
    lightOptionsMap.put(LightCode.CONE, 0.69);
    lightOptionsMap.put(LightCode.CUBE, 0.91);
    lightOptionsMap.put(LightCode.GAME_OBJECT, 0.71);
    lightOptionsMap.put(LightCode.NO_TAG, 0.75);
    lightOptionsMap.put(LightCode.WORKING, 0.61);
    lightOptionsMap.put(LightCode.READY_TO_SCORE, 0.87);
    lightOptionsMap.put(LightCode.OFF, 0.99); // 0.99 = black, is this equivalent to OFF?
  }

  public void toggleCode(LightCode light) {
    if (currentLightStatus == light) {
      currentLightStatus = LightCode.OFF;
    } else {
      currentLightStatus = light;
    }
    m_blinkin.set(lightOptionsMap.get(currentLightStatus));
  }

  public void setLight(LightCode light) {
    currentLightStatus = light;
    m_blinkin.set(lightOptionsMap.get(currentLightStatus));
  }

  /** Party mode switches from color to color to create a rainbow of lights */
  public void togglePartyMode() {
    this.partyMode = !this.partyMode;
  }

  @Override
  public void periodic() {
    /** NO_TAG is the only LightCode that requires blinking */
    if (currentLightStatus == LightCode.NO_TAG) {
      if (blinkingTimer >= blinkingPeriod) {
        if (blinkingColorOn) {
          m_blinkin.set(lightOptionsMap.get(LightCode.OFF));
        } else {
          m_blinkin.set(lightOptionsMap.get(LightCode.NO_TAG));
        }
        blinkingColorOn = !blinkingColorOn;
      }
      blinkingTimer++;
    } else {
      blinkingTimer = 0;
    }

    /* if party mode should be running, then start a party */
    if (partyMode) {
      m_blinkin.set(-0.97);
    }
  }
}
