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

    public Intake(Robot robot) {
        rightWheel.follow(leftWheel, true);
        this.robot = robot;
    }

    public void periodic() {
        if (state == States.OFF) {
            // stops motor movement and closes claw
            claw.set(true);
            leftWheel.stopMotor();
            // shift to intakes
            if (robot.secondaryController.getXButtonPressed()) {
                state = States.INTAKE;
            }
        } else if (state == States.INTAKE) {
            // opens or closes claw based on cube/cone and spins rollers
            if (robot.secondaryController.getAButtonPressed()) {
                isCone = !isCone;
            }
            claw.set(isCone);
            leftWheel.stopMotor();
            // changes to hold once beam brake is interrupted
            if (!hasNoGamePiece.get()) {
                state = States.HOLD;
            }
            checkForOuttake();
        } else if (state == States.HOLD) {
            // stops motors without changing claw's open/closed status
            leftWheel.stopMotor();
            claw.set(isCone);
            checkForOuttake();
        } else if (state == States.OUTTAKE) {
            // outtakes any game piece being held
            leftWheel.set(-1);
            // once the piece isn't sensed the claw is turned "off" 
            if (hasNoGamePiece.get()) {
                state = States.OFF;
            }
        }
    }

    private void checkForOuttake() {
        if (robot.secondaryController.getBButtonPressed()) {
            state = States.OUTTAKE;
        }
    }

}
