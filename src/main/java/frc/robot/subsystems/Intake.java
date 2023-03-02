package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

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
    CANSparkMax leftWheel = new CANSparkMax(10, MotorType.kBrushless);
    CANSparkMax rightWheel = new CANSparkMax(11, MotorType.kBrushless);
    Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, 1);

    DigitalInput hasNoObject = new DigitalInput(1);

    Robot robot = null;
    IntakeStates state = IntakeStates.OFF;
    ScoringLevels level = ScoringLevels.OFF;
    
    public void reset() {
        state = IntakeStates.OFF;
    }

    private final double INTAKE_SPEED = 0.2;
    Timer releaseTimer = new Timer();


    private boolean isCone = false;
    private final double HOLD_SPEED = 0.05;

    public Intake(Robot robot) {
        leftWheel.restoreFactoryDefaults();
        rightWheel.restoreFactoryDefaults();
        
        leftWheel.setSmartCurrentLimit(15);
        rightWheel.setSmartCurrentLimit(15);

        rightWheel.follow(leftWheel, true);
        this.robot = robot;
    }

    public void periodic() {
        SmartDashboard.putBoolean("Is Cone?", isCone);
        claw.set(isCone);
        SmartDashboard.putBoolean(" Claw Solenoid Status", claw.get());
        String stateString = "";

        if (state == IntakeStates.OFF) {
            // stops motor movement and closes claw
            System.out.println("Intake Off");
            leftWheel.stopMotor();
            robot.arm.setState(ArmSetStates.OFF);

            stateString = "Off";
            // shift to intakes
            if (robot.primaryController.getBButtonPressed()) {
                System.out.println("B Button Pressed (switching to cube intake)");
                isCone = false;
                state = IntakeStates.INTAKE;
            } else if (robot.primaryController.getYButtonPressed()) {
                System.out.println("Y Button Pressed (switching to cone intake)");
                isCone = true;
                state = IntakeStates.INTAKE;
            }
        } else if (state == IntakeStates.INTAKE) {
            leftWheel.set(INTAKE_SPEED);
            System.out.println("In Intake State");

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
                System.out.println("Auto Transition");
                state = IntakeStates.HOLD;
            } else if (robot.isManual()) {
                System.out.println("Check Manual Transition");
                checkForAdvance(IntakeStates.HOLD);
            }
        } else if (state == IntakeStates.HOLD) {
            System.out.println("In Hold");
            // stops motors without changing claw's open/closed status
            leftWheel.set(HOLD_SPEED);
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
                leftWheel.set(HOLD_SPEED);
                checkForAdvance(IntakeStates.RELEASING_CONE);
            } else {
                if (!robot.arm.waitingForFloorIntake() && robot.arm.nearSetpoint()) {
                    // outtakes any game piece being held
                    releaseTimer.start();
                    if (releaseTimer.get() > 2 || level == ScoringLevels.LEVEL_ONE || (!isCone && level == ScoringLevels.LEVEL_TWO)) {
                        leftWheel.set(-INTAKE_SPEED);
                        robot.rgb.setState(RGBStates.Neutral);
                        releaseTimer.reset();
                        releaseTimer.stop();
                    }
                } else {
                    leftWheel.set(HOLD_SPEED);
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
}
