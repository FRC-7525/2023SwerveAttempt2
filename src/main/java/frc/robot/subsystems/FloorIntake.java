package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

enum FloorIntakeStates {
    OFF,
    ON
}

public class FloorIntake {
    private String stateString;

    private WPI_TalonFX motor = new WPI_TalonFX(14);
    private Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);

    FloorIntakeStates state = FloorIntakeStates.OFF;
    Robot robot = null;

    public FloorIntake(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        if (state == FloorIntakeStates.OFF) {
            stateString = "Off";
            solenoid.set(false);
            motor.set(0);
        } else if (state == FloorIntakeStates.ON) {
            stateString = "On";
            solenoid.set(true);
            motor.set(0.45);
        }

        SmartDashboard.putString("Floor Intake State", stateString);
    }

    public void setState(FloorIntakeStates state) {
        this.state = state;
    }
}
