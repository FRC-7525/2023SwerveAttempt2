package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Robot;

enum FloorIntakeStates {
    OFF,
    ON
}

public class FloorIntake {
    private WPI_TalonFX motor = new WPI_TalonFX(14);
    private Solenoid rSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
    private Solenoid lSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);

    FloorIntakeStates state = FloorIntakeStates.OFF;
    Robot robot = null;

    public FloorIntake(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        if (state == FloorIntakeStates.OFF) {
            rSolenoid.set(false);
            lSolenoid.set(false);
            motor.set(0);
        } else if (state == FloorIntakeStates.ON) {
            rSolenoid.set(true);
            lSolenoid.set(true);
            motor.set(0.355);
        }
    }
    public void setState(FloorIntakeStates state) {
        this.state = state;
    }
}
