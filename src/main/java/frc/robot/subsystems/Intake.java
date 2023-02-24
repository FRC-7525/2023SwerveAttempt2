package frc.robot.subsystems;

import javax.print.attribute.standard.Compression;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

enum IntakeStates {
    OFF,
    INTAKE,
    HOLD,
    OUTTAKE
}

enum ButtonLevelStates {
    OFF,
    LEVEL_ONE,
    LEVEL_TWO
}

public class Intake {
    CANSparkMax leftWheel = new CANSparkMax(11, MotorType.kBrushless);
    CANSparkMax rightWheel = new CANSparkMax(10, MotorType.kBrushless);
    Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, 1);

    DigitalInput hasNoCone = new DigitalInput(1);
    DigitalInput hasNoCube = new DigitalInput(2);

    Robot robot = null;
    IntakeStates state = IntakeStates.OFF;
    private final double intakeSpeed = -0.2;

    private boolean isCone = false;

    public Intake(Robot robot) {
        rightWheel.follow(leftWheel, true);
        this.robot = robot;
    }

    public void periodic() {
        SmartDashboard.putBoolean("Is Cone?", isCone);
        claw.set(isCone);
        SmartDashboard.putBoolean("Solenoid Status", claw.get());
        String stateString = "";
        

        if (state == IntakeStates.OFF) {
            // stops motor movement and closes claw
            leftWheel.stopMotor();
            robot.rgb.setState(RGBStates.Neutral);
            robot.arm.setState(ArmSetStates.OFF);

            stateString = "Off";
            // shift to intakes
            if (robot.secondaryController.getYButtonPressed()) {
                if (robot.secondaryController.getLeftBumperPressed()) {
                    robot.rgb.setState(RGBStates.Cube);
                } else {
                    isCone = false;
                    state = IntakeStates.INTAKE;
                }
            } else if (robot.secondaryController.getAButtonPressed()) {
                if (robot.secondaryController.getLeftBumperPressed()) {
                    robot.rgb.setState(RGBStates.Cone);
                } else {
                    isCone = true;
                    state = IntakeStates.INTAKE;
                }
            }
        } else if (state == IntakeStates.INTAKE) {
            leftWheel.set(intakeSpeed);
            System.out.println("in intake state");

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
            } else if (robot.secondaryController.getLeftStickButton()) {
                level = ButtonLevelStates.LEVEL_ONE;
            } else if (robot.secondaryController.getRightStickButton()){
                level = ButtonLevelStates.LEVEL_TWO;
            }

            // changes to hold once beam brake is interrupted
            if (((isCone && !hasNoCone.get()) || (!isCone && !hasNoCube.get())) && !robot.isManual()) {
                System.out.println("auto transition");
                state = IntakeStates.HOLD;
            } else if (robot.isManual()) {
                System.out.println("Check manual transition");
                checkForAdvance(IntakeStates.HOLD);
            }
        } else if (state == IntakeStates.HOLD) {
            // stops motors without changing claw's open/closed status
            leftWheel.set(-0.05);
            checkForAdvance(IntakeStates.OUTTAKE);
            robot.arm.setState(ArmSetStates.OFF);

            if (isCone) {
                stateString = "Holding Cone";
                robot.rgb.setState(RGBStates.Cone);
            } else {
                stateString = "Holding Cube";
                robot.rgb.setState(RGBStates.Cube); 
            }
        } else if (state == IntakeStates.OUTTAKE) {
            if (level == ButtonLevelStates.LEVEL_ONE) {
                robot.arm.setState(ArmSetStates.LEVEL_ONE);
            
            if (!robot.arm.waitingForFloorIntake() && robot.arm.nearSetpoint()) {
                // outtakes any game piece being held
                leftWheel.set(-intakeSpeed);
                robot.rgb.setState(RGBStates.Neutral);
                stateString = "Outaking Gamepiece Level One";
            } 
            } else if (level == ButtonLevelStates.LEVEL_TWO) {
            robot.arm.setState(ArmSetStates.LEVEL_TWO);
            if (!robot.arm.waitingForFloorIntake() && robot.arm.nearSetpoint()) {
                // outtakes any game piece being held
                leftWheel.set(-intakeSpeed);
                robot.rgb.setState(RGBStates.Neutral);
                stateString = "Outaking Gamepiece Level Two";
            } 
            } else {
                leftWheel.set(0);
            }
            
            // once the piece isn't sensed the claw is turned "off" 
            if (hasNoCube.get() && hasNoCone.get() && !robot.isManual()) {
                state = IntakeStates.OFF;
            } else if (robot.isManual()) {
                checkForAdvance(IntakeStates.OFF);
            }
            stateString = "Outaking Gamepiece";
        }

        SmartDashboard.putString("Intake State", stateString);
    

    private void checkForAdvance(IntakeStates next) {
        if (robot.secondaryController.getBButtonPressed()) {
            state = next;
        }
    }

    public boolean isCone() {
        return this.isCone;
    }
}
