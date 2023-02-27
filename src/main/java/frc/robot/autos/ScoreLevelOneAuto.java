package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ScoreLevelOne;


public class ScoreLevelOneAuto extends SequentialCommandGroup {    
    public ScoreLevelOneAuto(Robot robot, boolean isCone) {
        addCommands(
            new ScoreLevelOne(robot, isCone)
        );
    }
}