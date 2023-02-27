package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    Swerve swerve;
    double elevationAngle;
    double output;

    final double platformMaxAngle = 10;
    final double balanceSpeed = 0.0684;
    final double polyCoeff = 1.5;

    public AutoBalance(Swerve swerve) {
        this.swerve = swerve;
        output = 0;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        elevationAngle = swerve.getElevationAngle();
    }

    @Override
    public void execute() {
        elevationAngle = swerve.getElevationAngle();
        output = updateDrive();
        SmartDashboard.putNumber("Balancing Speed", output);
        swerve.drive(new Translation2d(output, 0).times(Constants.Swerve.maxSpeed), 0, false, false);
    }

    private double updateDrive() {
        return -signOf(elevationAngle) * (Math.pow(polyCoeff * (Math.abs(elevationAngle) / platformMaxAngle), 2)) * balanceSpeed;
    }


    private int signOf(double num) {
        if(num < 0) {
            return -1;
        } else if (num > 0) {
            return 1;
        } else {
            return 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}