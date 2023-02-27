// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.DriveBackwardsAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;


public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs = new CTREConfigs();
    private Swerve swerve = new Swerve();

    private boolean toggleFieldRelative = false;

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
    public PneumaticHub ph = new PneumaticHub();
    public Arm arm = new Arm(this);
    public Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    private final SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();

    public boolean isManual() {
        return true;
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
        chooser.setDefaultOption("Drive Backwards", new DriveBackwardsAuto(swerve));
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
        ph.enableCompressorAnalog(80, 120);
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
        CommandScheduler.getInstance().schedule(chooser.getSelected());
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. 
     * @return */
    @Override
    public void teleopPeriodic() {
        if (secondaryController.getRightBumperPressed()) {
            isManual = !isManual;
        }

        SmartDashboard.putBoolean("Manual Mode", this.isManual());


        /* Drive */
        if (primaryController.getXButtonPressed()) {
            toggleFieldRelative = !toggleFieldRelative;
        }

        if (primaryController.getAButtonPressed()) {
            swerve.zeroGyro();
        }

        double translationVal = -MathUtil.applyDeadband(Swerve.squareInput(primaryController.getLeftY()), Constants.stickDeadband);
        double rotationVal = -MathUtil.applyDeadband(Swerve.squareInput(primaryController.getRightX()), Constants.stickDeadband);
        //rotationVal += translationVal * 0.01;
        rotationVal *= 0.75;

        double strafeVal = -MathUtil.applyDeadband(Swerve.squareInput(primaryController.getLeftX()), Constants.stickDeadband);

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
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
