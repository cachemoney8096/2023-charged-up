package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoChargeStationBalance extends CommandBase{
    private final DriveSubsystem drive;
    private final WPI_Pigeon2 gyro;
    private final double NOT_MOVING_IN_Y = 0;
    private final double NOT_ROTATING = 0;
    private final boolean ROBOT_RELATIVE = false;


    public AutoChargeStationBalance(DriveSubsystem drive){
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
        double velocityFromAngleDeg = pitchDeg * Calibrations.CHARGE_STATION_PITCH_DEGREES_TO_VELOCITY;

        double deadbandedVelocityFromAngleDeg = MathUtil.applyDeadband(velocityFromAngleDeg, Calibrations.CHARGE_STATION_DEADBAND_VELOCITY);

        double matchTime = DriverStation.getMatchTime();
        
        // stop driving (and thus set x) if there is less than one second left in auton
        drive.drive(matchTime > 1 ? deadbandedVelocityFromAngleDeg : 0, NOT_MOVING_IN_Y, NOT_ROTATING, ROBOT_RELATIVE);
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
