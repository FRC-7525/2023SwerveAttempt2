package frc.robot.autos;

import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAndDriveFurther extends SequentialCommandGroup {
    public ScoreAndDriveFurther(Robot robot, Swerve swerve){
        addCommands(
            new ScoreLevelThreeAuto(robot),
            new StraightMove(swerve, -4, true)
        );
    }
}