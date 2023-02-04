package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO do this subsystem
public class Lights extends SubsystemBase {
  private LightCodes currentLightStatus = LightCodes.OFF;

  public Lights() {}

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
  }

  public void toggleCubeLight() {
    if (currentLightStatus == LightCodes.CUBE) {
      currentLightStatus = LightCodes.OFF;
    } else {
      currentLightStatus = LightCodes.CUBE;
    }
  }

  @Override
  public void periodic() {}
}
