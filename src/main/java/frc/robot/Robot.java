// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.BalanceAuto;
import frc.robot.autos.DoNothingAuto;
import frc.robot.autos.DriveOverChargeStation;
import frc.robot.autos.LeaveCommunityAndBalance;
import frc.robot.autos.ScoreGrabAndBalance;
import frc.robot.autos.ScoreLevelOneAndBackAuto;
import frc.robot.autos.ScoreLevelOneAuto;
import frc.robot.autos.ScoreLevelThreeAuto;
import frc.robot.autos.SideMove;
import frc.robot.autos.StraightMove;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs = new CTREConfigs();
    private Swerve swerve = new Swerve();

    private boolean toggleFieldRelative = true;

    public XboxController primaryController = new XboxController(0);
    public XboxController secondaryController = new XboxController(1);

    //public final PhotonCamera camera = new PhotonCamera("Swerve_Front");

    private static String ROTATION_SPEED_SD = "Rotation Speed";
    private static String FIELD_RELATIVE_SD = "Field RELATIVE";
    //private Arm arm = new Arm(this);
    private boolean isManual = false;

    public FloorIntake floorIntake = new FloorIntake(this);
    public RGB rgb = new RGB(this);
    public Intake intake = new Intake(this);
    public Arm arm = new Arm(this);
    public Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    private final SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();


    public boolean isManual() {
        return true;
    }

    private void reset() {
        arm.reset();
        intake.reset();
        floorIntake.reset();
        rgb.reset();
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {        
        SmartDashboard.putNumber(ROTATION_SPEED_SD, 1);
        SmartDashboard.putBoolean(FIELD_RELATIVE_SD, toggleFieldRelative);
        CameraServer.startAutomaticCapture();

        PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath("New Path", new PathConstraints(1.0, 1.0));
        chooser.setDefaultOption("Drive Backwards", new StraightMove(swerve, -3, true));
        chooser.addOption("Lv3 Cube, Backwards, Auto Balance", new BalanceAuto(this, swerve));
        chooser.addOption("Do Nothing", new DoNothingAuto());
        chooser.addOption("Score Level One Cube", new ScoreLevelOneAuto(this, false));
        chooser.addOption("Score Level One Cone", new ScoreLevelOneAuto(this, true));
        chooser.addOption("Score Level One Cone and Drive Back", new ScoreLevelOneAndBackAuto(this, swerve, true));
        chooser.addOption("Score Level Three Cube", new ScoreLevelThreeAuto(this));
        //chooser.addOption("Move Right", new SideMove(swerve, 2, true));
        chooser.addOption("Move Left", new SideMove(swerve, -2, false));
        chooser.addOption("Leave Community and Balance", new LeaveCommunityAndBalance(this, swerve));
        chooser.addOption("Drive over charge station, then balance.", new DriveOverChargeStation(this, swerve));
        //chooser.addOption("Score, Grab, and Balance", new ScoreGrabAndBalance(this, swerve));
        chooser.addOption("PathWeaver", followTrajectoryCommand(ppTrajectory, true));

        SmartDashboard.putData("Auto Chooser", chooser);
        
        reset();
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
        compressor.enableAnalog(80, 120);
        SmartDashboard.putBoolean(FIELD_RELATIVE_SD, toggleFieldRelative);
        SmartDashboard.putNumber("Pressure", compressor.getPressure());
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
        reset();
        CommandScheduler.getInstance().schedule(chooser.getSelected());
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        arm.periodic();
        intake.periodic();
        swerve.periodic();
        rgb.periodic();
        floorIntake.periodic();
        CommandScheduler.getInstance().run();
    }

    public void disabledAutonomous() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopInit() {
        intake.resetControllerChecks();
        swerve.setAngleAdjustment(180);
        // TODO: comment out once we have autos!
        //reset();
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during operator control. 
     * @return */
    @Override
    public void teleopPeriodic() {
        /*
        if (secondaryController.getRightBumperPressed()) {
            isManual = !isManual;
        }
        */

        SmartDashboard.putBoolean("Manual Mode", this.isManual());


        /* Drive */
        if (primaryController.getXButtonPressed()) {
            toggleFieldRelative = !toggleFieldRelative;
        }

        if (primaryController.getAButtonPressed()) {
            swerve.zeroYaw();
        }

        double translationVal = -MathUtil.applyDeadband(Swerve.squareInput(primaryController.getLeftY()), Constants.stickDeadband);
        double rotationVal = -MathUtil.applyDeadband(Swerve.squareInput(primaryController.getRightX()), Constants.stickDeadband);
        //rotationVal += translationVal * 0.01;
        double strafeVal = -MathUtil.applyDeadband(Swerve.squareInput(primaryController.getLeftX()), Constants.stickDeadband);
        
        if (primaryController.getLeftBumper()) {
            rotationVal *= 0.1;
            translationVal *= 0.2;
            strafeVal *= 0.2;
        } else {
            rotationVal *= 0.75;
        }

        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                toggleFieldRelative,
                false);

        
        /* Vision */
        /* 
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();

        List<Double> ids = new ArrayList<Double>();
        for (PhotonTrackedTarget target : targets) {
            ids.add(Double.valueOf(target.getFiducialId()));
        }

        SmartDashboard.putBoolean("Has Target", !targets.isEmpty());
        SmartDashboard.putNumberArray("Target IDs", ids.stream().mapToDouble(d -> d).toArray());
        */

        arm.periodic();
        intake.periodic();
        swerve.periodic();
        rgb.periodic();
        floorIntake.periodic();
    }

    @Override
    public void testInit() {
        reset();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        compressor.enableAnalog(119, 120);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   swerve.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 swerve::getPose, // Pose supplier
                 Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                 new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 swerve::setModuleStates, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             )
         );
     }
}
