package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lights extends SubsystemBase {
  private LightCodes currentLightStatus = LightCodes.OFF;
  private AddressableLED m_led = new AddressableLED(RobotMap.LED_PWM_PORT);
  private AddressableLEDBuffer m_ledBuffer;
  private int spacing = 1;

  public Lights() {
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
  }

  public enum LightCodes {
    CONE, // Solid Yellow
    CUBE, // Solid Purple
    GAME_OBJECT, // Solid Green
    NO_TAG, // Blinking Green
    WORKING, // Solid Red
    READY_TO_SCORE, // Blue
    OFF
  }

  public void toggleConeLight() {
    if (currentLightStatus == LightCodes.CONE) {
      currentLightStatus = LightCodes.OFF;
    } else {
      currentLightStatus = LightCodes.CONE;
    }
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void toggleCubeLight() {
    if (currentLightStatus == LightCodes.CUBE) {
      currentLightStatus = LightCodes.OFF;
    } else {
      currentLightStatus = LightCodes.CUBE;
    }
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void toggleGameObjectLight() {
    if (currentLightStatus == LightCodes.GAME_OBJECT) {
      currentLightStatus = LightCodes.OFF;
    } else {
      currentLightStatus = LightCodes.GAME_OBJECT;
    }
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void toggelNoTagLight() {
    if (currentLightStatus == LightCodes.NO_TAG) {
      currentLightStatus = LightCodes.OFF;
    } else {
      currentLightStatus = LightCodes.NO_TAG;
    }
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void toggleWorkingLight() {
    if (currentLightStatus == LightCodes.WORKING) {
      currentLightStatus = LightCodes.OFF;
    } else {
      currentLightStatus = LightCodes.WORKING;
    }
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void toggleReadyToScoreLight() {
    if (currentLightStatus == LightCodes.READY_TO_SCORE) {
      currentLightStatus = LightCodes.OFF;
    } else {
      currentLightStatus = LightCodes.READY_TO_SCORE;
    }
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void toggleOff() {
    currentLightStatus = LightCodes.OFF;
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLight(LightCodes light) {
    currentLightStatus = light;
    setColor(currentLightStatus);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /** Sets the spacing between each LED so not all are lit up when applicable */
  public void setSpacing(int s) {
    spacing = s;
  }

  /** sets the color of the LEDs in RGB */
  public void setColor(LightCodes light) {
    int r = 0;
    int g = 0;
    int b = 0;

    if (light == LightCodes.CONE || light == LightCodes.CUBE || light == LightCodes.WORKING) {
      r = 255;
    }
    if (light == LightCodes.CONE || light == LightCodes.GAME_OBJECT || light == LightCodes.NO_TAG) {
      g = 255;
    }
    if (light == LightCodes.CUBE || light == LightCodes.READY_TO_SCORE) {
      b = 255;
    }
    if (light == LightCodes.OFF) {
      r = 0;
      g = 0;
      b = 0;
    }

    for (int i = 0; i < m_ledBuffer.getLength(); i = i + spacing) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }

  @Override
  public void periodic() {}
}
