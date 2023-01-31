// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.CANCoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;

    private Swerve swerve;

    private boolean toggleFieldRelative;

    XboxController controller = new XboxController(0);

    public final PhotonCamera camera = new PhotonCamera("Swerve_Front");

    private static String ROTATION_SPEED_SD = "Roation Speed";
    private static String FIELD_RELATIVE_SD = "Field RELATIVE";
    private static String X_POSITION_SD = "X Position";
    private static String Y_POSITION_SD = "Y Position";

    


    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        ctreConfigs = new CTREConfigs();
        swerve = new Swerve();
        
        SmartDashboard.putNumber(ROTATION_SPEED_SD, 1);
        SmartDashboard.putBoolean(FIELD_RELATIVE_SD, toggleFieldRelative);
    }
       

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        // CommandScheduler.getInstance().run();

        SmartDashboard.putBoolean(FIELD_RELATIVE_SD, toggleFieldRelative);

        swerve.periodic();

        Pose2d pose = swerve.getPose();
        double x_position = pose.getX();
        double y_position = pose.getY();
        SmartDashboard.putNumber(X_POSITION_SD, x_position);
        SmartDashboard.putNumber(Y_POSITION_SD, y_position);
        
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        swerve.drive(
            new Translation2d(1, 0).times(0.45),
            0,
            false,
            false);
    }

    @Override
    public void teleopInit() {

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        /* Drive */
        if (controller.getXButtonPressed()) {
            toggleFieldRelative = !toggleFieldRelative;
        }

        if (controller.getAButtonPressed()) {
            swerve.zeroGyro();
        }

        double translationVal = -MathUtil.applyDeadband(Swerve.squareInput(controller.getLeftY()), Constants.stickDeadband);
        double rotationVal = -MathUtil.applyDeadband(Swerve.squareInput(controller.getRightX()), Constants.stickDeadband);
        rotationVal *= SmartDashboard.getNumber("Rotation Speed", 1);

        double strafeVal = -MathUtil.applyDeadband(Swerve.squareInput(controller.getLeftX()), Constants.stickDeadband);

        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                toggleFieldRelative,
                false);

        
        /* Vision */
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();

        List<Double> ids = new ArrayList<Double>();
        for (PhotonTrackedTarget target : targets) {
            ids.add(Double.valueOf(target.getFiducialId()));
        }

        SmartDashboard.putBoolean("Has Target", !targets.isEmpty());
        SmartDashboard.putNumberArray("Target IDs", ids.stream().mapToDouble(d -> d).toArray());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
