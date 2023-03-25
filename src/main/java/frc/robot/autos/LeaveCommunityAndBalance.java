package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;

public class LeaveCommunityAndBalance extends SequentialCommandGroup {    
    public LeaveCommunityAndBalance(Robot robot, Swerve swerve) {
        addCommands(
            new StraightMove(swerve, -4.5, true),
            new SideMove(swerve, -2.2, false),
            new StraightMove(swerve, 1.5, false),
            new AutoBalance(robot, swerve)
        );
    }
}
