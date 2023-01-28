// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.geometry_helpers.decomposition;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

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

  PhotonPoseEstimator photonPoseEstimator;

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
    SmartDashboard.putNumber("Speed", 1);
    SmartDashboard.putNumber("Rotation Speed", 1);
    Pose3d pose1 = new Pose3d(0,inToMeters(-20.9),0,new Rotation3d(0,0, 0));
    Pose3d pose2 = new Pose3d(0,inToMeters(-50.25),0,new Rotation3d(0,0, 0));
    Pose3d pose3 = new Pose3d(0,inToMeters(-84.7),0,new Rotation3d(0,0, 0));
    
    AprilTag tag1 = new AprilTag(1, pose1);
    AprilTag tag2 = new AprilTag(2, pose2);
    AprilTag tag3 = new AprilTag(3, pose3);

    double fieldLength = 2.8194;
    double fieldWidth = 2.8194;

    List<AprilTag> tags = new ArrayList<>();

    tags.add(tag1);
    tags.add(tag2);
    tags.add(tag3);

    AprilTagFieldLayout layout = new AprilTagFieldLayout(tags, fieldLength, fieldWidth);

    Transform3d cameraToBot = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0,0,0));

    photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, cameraToBot);

    
  }

  private double inToMeters(double inches) {
    return inches * 0.0254;
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
    //CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("Field Relative", toggleFieldRelative);

    swerve.periodic();

    Optional<EstimatedRobotPose> optionalPoseEstimate = photonPoseEstimator.update();
    EstimatedRobotPose poseEstimate = optionalPoseEstimate.get();

    List<Double> pose = decomposition.DecomposePose3d(poseEstimate.estimatedPose);    
    Double[]posArray = new Double [pose.size()];
    pose.toArray(posArray);

    List<Double> rotations = decomposition.Decompose_Rotation_3d(poseEstimate.estimatedPose.getRotation());
    Double[]rotationsArray = new Double [rotations.size()];
    rotations.toArray(rotationsArray);
    


    //put to SmartDashboard
    SmartDashboard.putNumberArray("Pose", posArray);
    SmartDashboard.putNumberArray("Rotations", rotationsArray);

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
    //choose auto program!
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (controller.getXButtonPressed()) {
      toggleFieldRelative = !toggleFieldRelative;
    }

    double translationVal = -MathUtil.applyDeadband(Swerve.squareInput(controller.getLeftY()), Constants.stickDeadband);
    double rotationVal = -SmartDashboard.getNumber("Rotation Speed", 1) * MathUtil.applyDeadband(Swerve.squareInput(controller.getRightX()), Constants.stickDeadband);
    double strafeVal = -MathUtil.applyDeadband(Swerve.squareInput(controller.getLeftX()), Constants.stickDeadband);

    /* Drive */
    swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        toggleFieldRelative,
        false);

    var result = camera.getLatestResult();

    boolean hasTargets = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();
    
    List<Double> ids = new ArrayList<Double>();
    if (hasTargets) {
      for (PhotonTrackedTarget target : targets) {
        ids.add(Double.valueOf(target.getFiducialId()));
        
        
      }
    }
    
    SmartDashboard.putBoolean("Has Target", hasTargets);
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
