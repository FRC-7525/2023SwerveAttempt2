package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


enum RGBStates {
    Cone,
    Cube,
    Neutral
}

public class RGB {
    private String stateString;
    Robot robot = null;
    StringLogEntry rgbStateLog;
    PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    public RGB(Robot robot) {
        this.robot = robot;

        DataLog log = DataLogManager.getLog();
        rgbStateLog = new StringLogEntry(log, "/rgb/state");
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
            pdh.setSwitchableChannel(true);
            stateString = "Cone";
        } else if (state == RGBStates.Cube) {
            pdh.setSwitchableChannel(false);
            stateString = "Cube";
        } else if (state == RGBStates.Neutral) {
            stateString = "Neutral";
            pdh.setSwitchableChannel(false);
        }

        if (robot.secondaryController.getLeftBumperPressed()) {
            state = RGBStates.Cone;
        } else if (robot.secondaryController.getRightBumperPressed()) {
            state = RGBStates.Cube;
        }

        SmartDashboard.putString("RGB State", stateString);
        rgbStateLog.append(stateString);
    }
}





