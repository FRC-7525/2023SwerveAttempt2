package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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

    DigitalInput hasNoGamePiece = new DigitalInput(0);

    Robot robot;
    
    States state = States.OFF;

    boolean isCone = false;
    boolean isManual = false;

    public Intake(Robot robot) {
        rightWheel.follow(leftWheel, true);
        this.robot = robot;
    }

    public void periodic() {
        if (robot.secondaryController.getYButtonPressed()) {
            isManual = true;
        }
        
        if (state == States.OFF) {
            // stops motor movement and closes claw
            claw.set(true);
            leftWheel.stopMotor();

            // shift to intakes
            checkForIntake();
        } else if (state == States.INTAKE) {
            claw.set(isCone);
            leftWheel.set(.2);

            // opens or closes claw based on cube/cone and spins rollers
            checkForIntake();
            // changes to hold once beam brake is interrupted
            if (!hasNoGamePiece.get() && !isManual) {
                state = States.HOLD;
            } else if (isManual) {
                checkForAdvance(States.HOLD);
            }
        } else if (state == States.HOLD) {
            // stops motors without changing claw's open/closed status
            leftWheel.stopMotor();
            claw.set(isCone);
            checkForAdvance(States.OUTTAKE);
        } else if (state == States.OUTTAKE) {
            // outtakes any game piece being held
            leftWheel.set(-.2);
            // once the piece isn't sensed the claw is turned "off" 
            if (hasNoGamePiece.get() && !isManual) {
                state = States.OFF;
            } else if (isManual) {
                checkForAdvance(States.OFF);
            }
        }
    }

    private void checkForIntake() {
        if (robot.secondaryController.getXButtonPressed()) {
            isCone = false;
            state = States.INTAKE;
        }
        if (robot.secondaryController.getAButtonPressed()) {
            isCone = true;
            state = States.INTAKE;
        }
    }

    private void checkForAdvance(States next) {
        if (robot.secondaryController.getBButtonPressed()) {
            state = next;
        }
    }

}
