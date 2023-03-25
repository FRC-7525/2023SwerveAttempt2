package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

public class IntakeFloorCube extends SequentialCommandGroup {
    
    Robot robot = null;

    public IntakeFloorCube(Robot robot) {
        this.robot = robot;
    }

    

}
