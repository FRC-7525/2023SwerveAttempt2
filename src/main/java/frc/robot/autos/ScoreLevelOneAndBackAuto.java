package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ScoreLevelOne;
import frc.robot.subsystems.Swerve;


public class ScoreLevelOneAndBackAuto extends SequentialCommandGroup {    
    public ScoreLevelOneAndBackAuto(Robot robot, Swerve swerve, boolean isCone) {
        addCommands(
            new ScoreLevelOne(robot, isCone),
            new StraightMove(swerve, -3, true)
        );
    }
}