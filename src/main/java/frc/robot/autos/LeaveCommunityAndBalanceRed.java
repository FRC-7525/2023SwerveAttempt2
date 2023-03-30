package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ScoreLevelThree;

public class LeaveCommunityAndBalanceRed extends SequentialCommandGroup {    
    public LeaveCommunityAndBalanceRed(Robot robot, Swerve swerve) {
        addCommands(
            new ScoreLevelThree(robot),
            new StraightMove(swerve, -4.5, true),
            new SideMove(swerve, 1.6, true),
            new StraightMove(swerve, 2.5, false),
            new AutoBalance(robot, swerve)
        );
    }
}
