package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ScoreLevelThree;


public class ScoreLevelThreeAuto extends SequentialCommandGroup {    
    public ScoreLevelThreeAuto(Robot robot) {
        addCommands(
            new ScoreLevelThree(robot)
        );
    }
}