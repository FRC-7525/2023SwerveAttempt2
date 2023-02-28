package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class BalanceAuto extends SequentialCommandGroup {
    public BalanceAuto(Robot robot, Swerve swerve){
        AutoBalance balanceCommand = new AutoBalance(robot, swerve);

        addCommands(
            balanceCommand
        );
    }
}