package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ScoreLevelThree;

public class DriveOverChargeStation extends SequentialCommandGroup {
    public DriveOverChargeStation(Robot robot, Swerve swerve){
        addCommands(
            new ScoreLevelThree(robot),
            new StraightMove(swerve, -4.5, true),
            new StraightMove(swerve, 1.7, false),
            new AutoBalance(robot, swerve)
        );
    }
}
