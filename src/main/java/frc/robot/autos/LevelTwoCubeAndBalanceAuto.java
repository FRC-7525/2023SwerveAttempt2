package frc.robot.autos;

import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ScoreLevelTwoCube;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LevelTwoCubeAndBalanceAuto extends SequentialCommandGroup {
    public LevelTwoCubeAndBalanceAuto(Robot robot, Swerve swerve){
        addCommands(
            new ScoreLevelTwoCube(robot),
            new StraightMove(swerve, -3, true),
            new AutoBalance(robot, swerve)
        );
    }
}