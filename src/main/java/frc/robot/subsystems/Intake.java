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
    LEVEL_TWO
}

public class Intake {
    CANSparkMax leftWheel = new CANSparkMax(10, MotorType.kBrushless);
    CANSparkMax rightWheel = new CANSparkMax(11, MotorType.kBrushless);
    Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, 1);

    DigitalInput hasNoObject = new DigitalInput(1);

    Robot robot = null;
    IntakeStates state = IntakeStates.OFF;
    ScoringLevels level = ScoringLevels.OFF;

    Timer releasing_cone_timer = new Timer();

    private final double intakeSpeed = 0.2;

    private boolean isCone = false;

    public Intake(Robot robot) {
        leftWheel.restoreFactoryDefaults();
        rightWheel.restoreFactoryDefaults();

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
            robot.rgb.setState(RGBStates.Neutral);
            robot.arm.setState(ArmSetStates.OFF);

            stateString = "Off";
            // shift to intakes
            if (robot.secondaryController.getYButtonPressed()) {
                System.out.println("Y Button Pressed (switching to cube intake)");
                if (robot.secondaryController.getLeftBumperPressed()) {
                    robot.rgb.setState(RGBStates.Cube);
                } else {
                    isCone = false;
                    state = IntakeStates.INTAKE;
                }
            } else if (robot.secondaryController.getAButtonPressed()) {
                System.out.println("A Button Pressed (switching to cone intake)");
                if (robot.secondaryController.getLeftBumperPressed()) {
                    robot.rgb.setState(RGBStates.Cone);
                } else {
                    isCone = true;
                    state = IntakeStates.INTAKE;
                }
            }
        } else if (state == IntakeStates.INTAKE) {
            leftWheel.set(intakeSpeed);
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
            if (robot.secondaryController.getYButtonPressed()) {
                if (isCone) {
                    isCone = false;
                } else {
                    state = IntakeStates.OFF;
                }
            } else if (robot.secondaryController.getAButtonPressed()) {
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
            leftWheel.set(0.05);
            robot.arm.setState(ArmSetStates.OFF);

            if (isCone) {
                stateString = "Holding Cone";
                robot.rgb.setState(RGBStates.Cone);
            } else {
                stateString = "Holding Cube";
                robot.rgb.setState(RGBStates.Cube); 
            }

            if (robot.secondaryController.getPOV() == 90) {
                // Right on D-Pad
                level = ScoringLevels.LEVEL_TWO;
                state = IntakeStates.OUTTAKE;
            } else if (robot.secondaryController.getPOV() == 180) {
                // Down on D-Pad
                level = ScoringLevels.LEVEL_ONE;
                state = IntakeStates.OUTTAKE;
            }
        } else if (state == IntakeStates.OUTTAKE) {            
            if (level == ScoringLevels.LEVEL_ONE) {
                robot.arm.setState(ArmSetStates.LEVEL_ONE);
                stateString = "Outtaking Gamepiece Level One"; 
            } else if (level == ScoringLevels.LEVEL_TWO) {
                robot.arm.setState(ArmSetStates.LEVEL_TWO);
                stateString = "Outtaking Gamepiece Level Two";
            }

            if (isCone && level == ScoringLevels.LEVEL_TWO) {
                leftWheel.set(0);
                checkForAdvance(IntakeStates.RELEASING_CONE);
            } else {
                if (!robot.arm.waitingForFloorIntake() && robot.arm.nearSetpoint()) {
                    // outtakes any game piece being held
                    leftWheel.set(-intakeSpeed);
                    robot.rgb.setState(RGBStates.Neutral);
                } else {
                    leftWheel.set(0);
                }

                // once the piece isn't sensed the claw is turned "off"
                if (hasNoObject.get() && !robot.isManual()) {
                    state = IntakeStates.OFF;
                } else if (robot.isManual()) {
                    checkForAdvance(IntakeStates.OFF);
                }
            }
        } else if (state == IntakeStates.RELEASING_CONE) {
            stateString = "Releasing Cone (B to turn off)";
            claw.set(false);
            checkForAdvance(IntakeStates.OFF);
        }

        SmartDashboard.putString("Intake State", stateString);
    }
    

    private void checkForAdvance(IntakeStates next) {
        if (robot.secondaryController.getBButtonPressed()) {
            state = next;
        }
    }

    public boolean isCone() {
        return this.isCone;
    }
}
