package frc.robot.autos;

import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BalanceAuto extends SequentialCommandGroup {
    public BalanceAuto(Robot robot, Swerve swerve){
        addCommands(
            new ScoreLevelThreeAuto(robot),
            new StraightMove(swerve, -2.8, true),
            new AutoBalance(robot, swerve)
        );
    }
}