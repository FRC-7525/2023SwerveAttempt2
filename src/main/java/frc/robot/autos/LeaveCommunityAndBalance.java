package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ScoreLevelThree;

public class LeaveCommunityAndBalance extends SequentialCommandGroup {    
    public LeaveCommunityAndBalance(Robot robot, Swerve swerve) {
        addCommands(
            new ScoreLevelThree(robot),
            new StraightMove(swerve, -4.5, true),
            (DriverStation.getAlliance() != DriverStation.Alliance.Blue?
                new SideMove(swerve, -2.2, false):
                new SideMove(swerve, 2.2, true)),
            new StraightMove(swerve, 1.8, false),
            new AutoBalance(robot, swerve)
        );
    }
}
