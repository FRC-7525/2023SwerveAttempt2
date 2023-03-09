package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.subsystems.Swerve;

public class AutoBalance2 extends CommandBase {
    Swerve swerve = null;
    Robot robot = null;
    PIDController angleController = new PIDController(0.005, 0, 0);
    double offset = -6.65;
    double angle = 0;
    private boolean isReversed;


    public AutoBalance2(Robot robot, Swerve swerve, boolean isReversed) {
        this.robot = robot;
        this.swerve = swerve;
        addRequirements(swerve);
        
    }

    @Override
    public void initialize() {
        angleController.setSetpoint(0);
        this.isReversed = false;
    }

    @Override
    public void execute() {
      
        if (isReversed == true) {
            angle = -1 * swerve.getElevationAngle();
        }

        double elevationAngle = angle - offset;
        //elevationAngle = Rotation2d.fromRadians(elevationAngle).getDegrees();
        SmartDashboard.putNumber("Angle", elevationAngle);
        double output = angleController.calculate(elevationAngle);
        SmartDashboard.putNumber("Balancing Speed", output);
        swerve.drive(new Translation2d(output, 0).times(Constants.Swerve.maxSpeed), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}