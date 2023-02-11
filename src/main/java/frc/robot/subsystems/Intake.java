package frc.robot.subsystems;

import java.lang.Thread.State;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

enum States {
    OFF,
    INTAKE,
    HOLD,
    OUTTAKE
}

public class Intake {
    CANSparkMax leftWheel = new CANSparkMax(11, MotorType.kBrushless);
    CANSparkMax rightWheel = new CANSparkMax(10, MotorType.kBrushless);
    Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, 1);

    DigitalInput hasNoCone = new DigitalInput(0);
    DigitalInput hasNoCube = new DigitalInput(1);

    Robot robot = null;
    States state = States.OFF;

    boolean isCone = false;

    public Intake(Robot robot) {
        rightWheel.follow(leftWheel, true);
        this.robot = robot;
    }

    public void periodic() {

        SmartDashboard.putBoolean("isCone", isCone);
        String stateString = "";

        if (state == States.OFF) {
            // stops motor movement and closes claw
            claw.set(true);
            leftWheel.stopMotor();

            stateString = "Off";
            // shift to intakes
            if (robot.secondaryController.getXButtonPressed()) {
                isCone = false;     
                state = States.INTAKE;
            } else if (robot.secondaryController.getAButtonPressed()) {
                isCone = true;
                state = States.INTAKE;
            }
        } else if (state == States.INTAKE) {
            claw.set(isCone);
            leftWheel.set(.2);

            if (isCone) {
                stateString = "Intaking Cone";
            } else {
                stateString = "Intaking Cube";
            }

            // opens or closes claw based on cube/cone and spins rollers
            if (robot.secondaryController.getXButtonPressed()) {
                if (isCone) {
                    isCone = false;
                } else {
                    state = States.OFF;
                }
            }
            if (robot.secondaryController.getAButtonPressed()) {
                if (!isCone) {
                    isCone = true;
                } else {
                    state = States.OFF;
                }
            }

            // changes to hold once beam brake is interrupted
            if (((isCone && !hasNoCone.get()) || (!isCone && !hasNoCube.get())) && !robot.isManual()) {
                state = States.HOLD;
            } else if (robot.isManual()) {
                checkForAdvance(States.HOLD);
            }
        } else if (state == States.HOLD) {
            // stops motors without changing claw's open/closed status
            leftWheel.stopMotor();
            claw.set(isCone);
            checkForAdvance(States.OUTTAKE);

            if (isCone) {
                stateString = "Holding Cone";
            } else {
                stateString = "Holding Cube";
            }


        } else if (state == States.OUTTAKE) {
            // outtakes any game piece being held
            leftWheel.set(-.2);
            
            // once the piece isn't sensed the claw is turned "off" 
            if (hasNoCube.get() && hasNoCone.get() && !robot.isManual()) {
                state = States.OFF;
            } else if (robot.isManual()) {
                checkForAdvance(States.OFF);
            }
            stateString = "Outaking Gamepiece";
        }

        SmartDashboard.putString("Intake State", stateString);
    }

    private void checkForAdvance(States next) {
        if (robot.secondaryController.getBButtonPressed()) {
            state = next;
        }
    }
}
