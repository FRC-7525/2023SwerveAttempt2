package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;


enum FloorIntakeStates {
    OFF,
    ON,
    OUTTAKE,
    DOWN_HOLD
}

public class FloorIntake {
    private String stateString;

    private WPI_TalonFX motor = new WPI_TalonFX(14);
    private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    public final double FLOOR_INTAKE_SPEED = 0.5;

    StringLogEntry floorIntakeStateLog;

    FloorIntakeStates state = FloorIntakeStates.OFF;
    Robot robot = null;

    public void reset() {
        state = FloorIntakeStates.OFF;
    }

    public FloorIntake(Robot robot) {
        this.robot = robot;
        // Set up custom log entries
        DataLog log = DataLogManager.getLog();
        floorIntakeStateLog = new StringLogEntry(log, "/floorintake/state");
    }

    public void periodic() {
        if (state == FloorIntakeStates.OFF) {
            stateString = "Off";
            solenoid.set(false);
            motor.set(0);

        } else if (state == FloorIntakeStates.ON) {
            stateString = "On (Wheels On)";
            solenoid.set(true);
            
            if (robot.primaryController.getRightBumper()) {
                motor.set(-FLOOR_INTAKE_SPEED);
                floorIntakeStateLog.append("Floor intake wheels inverting");

            } else {
                motor.set(FLOOR_INTAKE_SPEED);
            }
        } else if (state == FloorIntakeStates.OUTTAKE) {
            stateString = "Outtake Cube";
            solenoid.set(true);
            motor.set(-FLOOR_INTAKE_SPEED);
        } else if (state == FloorIntakeStates.DOWN_HOLD) {
            stateString = "On (Wheels Off)";
            solenoid.set(true);
            motor.set(0);
        } 

        SmartDashboard.putString("Floor Intake State", stateString);
        floorIntakeStateLog.append(stateString);
    }

    public void setState(FloorIntakeStates state) {
        this.state = state;
    }
}
