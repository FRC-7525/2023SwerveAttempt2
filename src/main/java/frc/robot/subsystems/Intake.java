package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

enum IntakeStates {
    OFF,
    INTAKE,
    HOLD,
    OUTTAKE,
    RELEASING_CONE
}

enum ScoringLevels {
    OFF,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE
}

public class Intake {
    CANSparkMax wheel = new CANSparkMax(10, MotorType.kBrushless);
    Solenoid claw = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    DigitalInput hasNoObject = new DigitalInput(1);

    Robot robot = null;
    IntakeStates state = IntakeStates.OFF;
    ScoringLevels level = ScoringLevels.OFF;

    StringLogEntry intakeStateLog;
    
    public void reset() {
        state = IntakeStates.OFF;
    }

    private final double INTAKE_SPEED = 0.5;
    Timer releaseTimer = new Timer();


    private boolean isCone = false;
    private final double HOLD_SPEED = 0.05;

    public Intake(Robot robot) {
        wheel.restoreFactoryDefaults();
        this.robot = robot;

        DataLog log = DataLogManager.getLog();
        intakeStateLog = new StringLogEntry(log, "/arm/state");
    }

    public void periodic() {
        SmartDashboard.putBoolean("Is Cone?", isCone);
        claw.set(isCone);
        SmartDashboard.putBoolean(" Claw Solenoid Status", claw.get());
        String stateString = "";

        if (state == IntakeStates.OFF) {
            // stops motor movement and closes claw
            wheel.stopMotor();
            robot.arm.setState(ArmSetStates.OFF);

            stateString = "Off";
            // shift to intakes
            if (robot.primaryController.getBButtonPressed()) {
                isCone = false;
                state = IntakeStates.INTAKE;
            } else if (robot.primaryController.getYButtonPressed()) {
                isCone = true;
                state = IntakeStates.INTAKE;
            }
        } else if (state == IntakeStates.INTAKE) {
            wheel.set(INTAKE_SPEED);

            if (isCone) {
                stateString = "Intaking Cone";
                robot.rgb.setState(RGBStates.Cone);
                robot.arm.setState(ArmSetStates.CONE_ON);
            } else {
                stateString = "Intaking Cube";
                robot.rgb.setState(RGBStates.Cube);
                robot.arm.setState(ArmSetStates.CUBE_ON);
            }

            // opens or closes claw based on cube/cone and spins rollers
            if (robot.primaryController.getBButtonPressed()) {
                if (isCone) {
                    isCone = false;
                } else {
                    state = IntakeStates.OFF;
                }
            } else if (robot.primaryController.getYButtonPressed()) {
                if (!isCone) {
                    isCone = true;
                } else {
                    state = IntakeStates.OFF;
                }
            }

            // changes to hold once beam brake is interrupted
            if (!hasNoObject.get() && !robot.isManual()) {
                state = IntakeStates.HOLD;
            } else if (robot.isManual()) {
                checkForAdvance(IntakeStates.HOLD);
            }
        } else if (state == IntakeStates.HOLD) {
            // stops motors without changing claw's open/closed status
            wheel.set(HOLD_SPEED);
            robot.arm.setState(ArmSetStates.OFF);

            if (isCone) {
                stateString = "Holding Cone";
                robot.rgb.setState(RGBStates.Cone);
            } else {
                stateString = "Holding Cube";
                robot.rgb.setState(RGBStates.Cube); 
            }

            if (robot.secondaryController.getPOV() == 90 || robot.secondaryController.getPOV() == 270) {
                // Left/Right on D-Pad
                level = ScoringLevels.LEVEL_TWO;
                state = IntakeStates.OUTTAKE;
            } else if (robot.secondaryController.getPOV() == 180) {
                // Down on D-Pad
                level = ScoringLevels.LEVEL_ONE;
                state = IntakeStates.OUTTAKE;
            } else if (robot.secondaryController.getPOV() == 0) {
                // Up on D-Pad, only for cubes
                if (!isCone) {
                    level = ScoringLevels.LEVEL_THREE;
                    state = IntakeStates.OUTTAKE;
                }
            }
        } else if (state == IntakeStates.OUTTAKE) { 
            // State setup           
            if (level == ScoringLevels.LEVEL_ONE) {
                robot.arm.setState(ArmSetStates.LEVEL_ONE);
                stateString = "Outtaking Gamepiece (Level One)"; 
            } else if (level == ScoringLevels.LEVEL_TWO) {
                robot.arm.setState(ArmSetStates.LEVEL_TWO);
                stateString = "Outtaking Gamepiece (Level Two)";
            } else if (level == ScoringLevels.LEVEL_THREE) {
                robot.arm.setState(ArmSetStates.LEVEL_THREE);
                stateString = "Outtaking Gamepiece (Level Three)";
            }

            if (isCone && level == ScoringLevels.LEVEL_TWO) {
                wheel.set(HOLD_SPEED);
                checkForAdvance(IntakeStates.RELEASING_CONE);
            } else {
                if (!robot.arm.waitingForFloorIntake() && robot.arm.nearSetpoint()) {
                    // outtakes any game piece being held
                    releaseTimer.start();
                    if (releaseTimer.get() > 0.3 || level == ScoringLevels.LEVEL_ONE || (!isCone && level == ScoringLevels.LEVEL_TWO)) {
                        wheel.set(-INTAKE_SPEED);
                        robot.rgb.setState(RGBStates.Neutral);
                        releaseTimer.reset();
                        releaseTimer.stop();
                    }
                } else {
                    wheel.set(HOLD_SPEED);
                    releaseTimer.reset();
                }

                // once the piece isn't sensed the claw is turned "off"
                checkForAdvance(IntakeStates.OFF);
            }
        } else if (state == IntakeStates.RELEASING_CONE) {
            stateString = "Releasing Cone (B to turn off)";
            claw.set(false);
            checkForAdvance(IntakeStates.OFF);
        }

        SmartDashboard.putString("Intake State", stateString);
        intakeStateLog.append(stateString);
        resetControllerChecks();
    }    

    private void checkForAdvance(IntakeStates next) {
        if (robot.secondaryController.getBButtonPressed()) {
            state = next;
        }
    }

    // This calls the "pressed" functions, to make sure the
    // pressed state isn't retained when the button is pressed
    // earlier on randomly
    public void resetControllerChecks() {
        robot.secondaryController.getBButtonPressed();
        robot.primaryController.getYButtonPressed();
        robot.primaryController.getBButtonPressed();
    }

    public boolean isCone() {
        return this.isCone;
    }

    // Autonomous Setups    
    public void setUpLevelOne(boolean isCone) {
        this.isCone = isCone;
        level = ScoringLevels.LEVEL_ONE;
        state = IntakeStates.OUTTAKE;
    }

    public void setUpLevelThree() {
        isCone = false;
        level = ScoringLevels.LEVEL_THREE;
        state = IntakeStates.OUTTAKE;
    }

    public void setUpLevelTwoCube() {
        isCone = false;
        level = ScoringLevels.LEVEL_TWO;
        state = IntakeStates.OUTTAKE;
    }

    public void setUpIntakeCube() {
        isCone = false;
        state = IntakeStates.INTAKE;
    }

    // Autonomous Reset

    public void resetIntakeCube() {
        isCone = false;
        state = IntakeStates.HOLD;
    }
}
