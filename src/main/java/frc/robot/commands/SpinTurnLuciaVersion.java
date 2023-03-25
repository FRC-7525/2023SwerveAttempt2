package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;


public class SpinTurnLuciaVersion extends CommandBase {    
    Swerve swerve = null;
    double finalYaw = 0;

    public SpinTurnLuciaVersion(Swerve swerve, double turnInDegrees) {
        this.finalYaw = swerve.getYaw().getDegrees() + turnInDegrees;
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(0, 0), 0.3, false, false);
    }

    @Override
    public boolean isFinished() {
        return (swerve.getYaw().getDegrees() == finalYaw);
    }
}
