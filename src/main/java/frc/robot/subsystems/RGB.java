package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import org.opencv.video.SparseOpticalFlow;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

enum RGBStates {
    Cone,
    Cube,
    Neutral
}

public class RGB {
    private String stateString;
    Spark RGBcontrol = new Spark(0);
    Robot robot = null;

    public RGB(Robot robot) {
        this.robot = robot;
    }

    RGBStates state = RGBStates.Neutral;
    

    public void setState(RGBStates state) {
        this.state = state;
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

        SmartDashboard.putString("RGB State", stateString);
    }
}





