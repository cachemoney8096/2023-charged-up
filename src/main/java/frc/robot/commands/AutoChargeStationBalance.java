package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Cal;
import frc.robot.Cal.AutoBalance;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Command to automatically remain balanced on the charge station during autonomous, using the
 * gyroscope.
 */
public class AutoChargeStationBalance extends CommandBase {
  private final DriveSubsystem drive;
  private final WPI_Pigeon2 gyro;
  private final double NOT_MOVING_IN_Y = 0;
  private final double NOT_ROTATING = 0;
  private final boolean ROBOT_RELATIVE = false;

  public AutoChargeStationBalance(DriveSubsystem drive) {
    this.drive = drive;
    this.gyro = drive.getGyro();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchDeg = gyro.getPitch();

    /** Velocity is [-1,1] */
    double normVelocity = pitchDeg * AutoBalance.CHARGE_STATION_PITCH_DEGREES_TO_NORM_VELOCITY;

    double deadbandedNormVelocity =
        MathUtil.applyDeadband(normVelocity, AutoBalance.CHARGE_STATION_DEADBAND_NORM_VELOCITY);

    /** Time remaining in current match period (auto or teleop) in seconds */
    double matchTime = DriverStation.getMatchTime();

    // stop driving (and thus set x) if there is less than one second left in auton
    if (matchTime > Cal.AutoBalance.SET_X_TIME_LEFT_SECONDS) {
      drive.drive(deadbandedNormVelocity, NOT_MOVING_IN_Y, NOT_ROTATING, ROBOT_RELATIVE);
    } else {
      drive.setX();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
