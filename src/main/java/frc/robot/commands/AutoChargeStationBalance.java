package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoChargeStationBalance extends CommandBase{
    private final DriveSubsystem drive;
    private final WPI_Pigeon2 gyro;

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
        double angle = gyro.getPitch();

        angle = MathUtil.applyDeadband(angle, Calibrations.CHARGE_STATION_DEADBAND_PITCH_DEGREES);

        double velocityFromAngle = angle * Calibrations.CHARGE_STATION_PITCH_DEGREES_TO_VELOCITY;

        drive.drive(velocityFromAngle, 0, 0, false);
        
        if (angle == 0){
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
