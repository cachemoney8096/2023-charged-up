package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SendableHelper {
  /**
   * Add a sendable to the dashboard under an existing sendable. The root sendable must be part of a
   * subsystem.
   *
   * @param builder Builder passed to the initSendable
   * @param parent Parent sendable object
   * @param child Sendable to add under the parent
   * @param name Name for the group the child will be added under
   */
  public static void addChild(
      SendableBuilder builder, Sendable parent, Sendable child, String name) {
    String subsystem = SendableRegistry.getSubsystem(parent);
    String parentName = SendableRegistry.getName(parent);
    SendableRegistry.addLW(child, subsystem, parentName + "/" + name);
  }
}
