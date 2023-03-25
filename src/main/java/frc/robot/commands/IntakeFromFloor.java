package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.autos.StraightMove;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

enum IntakeFromFloorState {
    INTAKE,
    RESET,
    DONE
}

public class IntakeFromFloor extends CommandBase {    
    Timer intakeTimer = new Timer();
    Robot robot = null;
    Swerve swerve;
    int time = 0;
    boolean forwardMove = false;
    double distance = 0;
    IntakeFromFloorState state = IntakeFromFloorState.INTAKE;

    public IntakeFromFloor(Robot robot, Swerve swerve, int time, boolean forwardMove, double distance) {
        this.robot = robot;
        this.swerve = swerve;
        this.time = time;
        this.forwardMove = forwardMove;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        robot.intake.setUpIntakeCube();
        intakeTimer.reset();
    }

    @Override
    public void execute() {
        if (state == IntakeFromFloorState.INTAKE) {
            intakeTimer.start();
            if (intakeTimer.get() > 3) {
                state = IntakeFromFloorState.RESET;
                robot.intake.reset();
                intakeTimer.reset();
            }
        } else if (state == IntakeFromFloorState.RESET) {
            intakeTimer.start();
            if (intakeTimer.get() > 2) {
                state = IntakeFromFloorState.DONE;
            }
        }

        if (forwardMove && state != IntakeFromFloorState.DONE) {
            new StraightMove(swerve, distance, false);
            forwardMove = false;
        }

        if (state == IntakeFromFloorState.DONE) {
            robot.intake.resetIntakeCube();
        }
    }

    public boolean isFinished() {
        return state == IntakeFromFloorState.DONE;
    }
}