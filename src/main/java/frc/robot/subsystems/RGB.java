package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;


enum RGBStates {
    Cone,
    Cube,
    Neutral
}

public class RGB {
    private String stateString;
    Spark RGBcontrol = new Spark(0);
    Robot robot = null;
    StringLogEntry RGBStateLog;

    public RGB(Robot robot) {
        this.robot = robot;

        DataLog log = DataLogManager.getLog();
        RGBStateLog = new StringLogEntry(log, "/RGB/state");
    }

    RGBStates state = RGBStates.Neutral;
    

    public void setState(RGBStates state) {
        this.state = state;
    }
    
    public void reset() {
        state = RGBStates.Neutral;
    }

    public void periodic() {
        if (state == RGBStates.Cone) {
            RGBcontrol.set(0.69);
            stateString = "Cone";
        } else if (state == RGBStates.Cube) {
            RGBcontrol.set(0.89);
            stateString = "Cube";
        } else if (state == RGBStates.Neutral) {
            stateString = "Neutral";
            RGBcontrol.set(0.99);
        }

        if (robot.secondaryController.getLeftBumperPressed()) {
            state = RGBStates.Cone;
        } else if (robot.secondaryController.getRightBumperPressed()) {
            state = RGBStates.Cube;
        }

        SmartDashboard.putString("RGB State", stateString);
        RGBStateLog.append(stateString);
    }
}





