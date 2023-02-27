package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

enum ScoreLevelOneState {
    SCORE,
    RETRACT,
    DONE
}

public class ScoreLevelOne extends CommandBase {
    Robot robot = null;
    boolean isCone;
    ScoreLevelOneState state = ScoreLevelOneState.SCORE;
    Timer stateTimer = new Timer();

    public ScoreLevelOne(Robot robot, boolean isCone) {
        this.isCone = isCone;
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.intake.setUpLevelOne(isCone);
        stateTimer.reset();
    }

    @Override
    public void execute() {
        if (state == ScoreLevelOneState.SCORE) {
            stateTimer.start();
            if (stateTimer.get() > 3) {
                state = ScoreLevelOneState.RETRACT;
                robot.intake.reset();
                stateTimer.reset();
            }
        } else if (state == ScoreLevelOneState.RETRACT) {
            stateTimer.start();
            if (stateTimer.get() > 2) {
                state = ScoreLevelOneState.DONE;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return state == ScoreLevelOneState.DONE;
    }
}