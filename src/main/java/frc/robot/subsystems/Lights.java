package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO do this subsystem
public class Lights extends SubsystemBase {
  private LightCodes currentLightStatus = LightCodes.OFF;

  public Lights() {}

  public enum LightCodes {
    CONE,
    CUBE,
    GAME_OBJECT,
    NO_TAG,
    WORKING,
    READY_TO_SCORE,
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
}
