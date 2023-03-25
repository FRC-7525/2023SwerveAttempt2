package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class HoldCube extends SequentialCommandGroup {  
    Robot robot = null;

    public HoldCube(Robot robot) {
        this.robot = robot;
    }

    

}