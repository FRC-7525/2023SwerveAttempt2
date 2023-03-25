package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;

public class DriveOverChargeStation extends SequentialCommandGroup {
    public DriveOverChargeStation(Robot robot, Swerve swerve){
        addCommands(
            new StraightMove(swerve, 4.5, false),
            new StraightMove(swerve, -1.7, true),
            new AutoBalance(robot, swerve)
        );
    }
}
