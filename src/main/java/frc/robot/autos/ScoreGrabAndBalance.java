package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.IntakeFromFloor;
import frc.robot.commands.SpinTurnGyro;

public class ScoreGrabAndBalance extends SequentialCommandGroup {    
    public ScoreGrabAndBalance(Robot robot, Swerve swerve) {
        addCommands(
           // new ScoreLevelThreeAuto(robot),
            new StraightMove(swerve, -6.0, true),
            new SpinTurnGyro(swerve, 180)
           // new IntakeFromFloor(robot, swerve, 1, true, 0.8),
            /*new StraightMove(swerve, -2.3, true),
            new SideMove(swerve, -2.7, false),
            new StraightMove(swerve, -1.5, true),
            new AutoBalance(robot, swerve)		*/
        );
    }
}
