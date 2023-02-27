package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DoNothingAuto extends SequentialCommandGroup {
    public DoNothingAuto() {
        addCommands(
            new InstantCommand(() -> System.out.println("Do Nothing Auto"))
        );
    }
}