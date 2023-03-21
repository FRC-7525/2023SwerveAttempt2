package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

public class MoveRight extends SequentialCommandGroup {    
    public MoveRight(Robot robot, Swerve swerve) {
        addCommands(
            new SideMove(swerve, 2.7, false)
        );
    }
}