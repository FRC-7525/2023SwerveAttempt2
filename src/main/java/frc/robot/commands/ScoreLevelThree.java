package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

enum ScoreLevelThreeState {
    SCORE,
    RETRACT,
    DONE
}

public class ScoreLevelThree extends CommandBase {
    Robot robot = null;
    boolean isCone;
    ScoreLevelThreeState state = ScoreLevelThreeState.SCORE;
    Timer stateTimer = new Timer();

    public ScoreLevelThree(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.intake.setUpLevelThree();
        stateTimer.reset();
    }

    @Override
    public void execute() {
        if (state == ScoreLevelThreeState.SCORE) {
            stateTimer.start();
            if (stateTimer.get() > 4) {
                state = ScoreLevelThreeState.RETRACT;
                robot.intake.reset();
                stateTimer.reset();
            }
        } else if (state == ScoreLevelThreeState.RETRACT) {
            stateTimer.start();
            if (stateTimer.get() > 3) {
                state = ScoreLevelThreeState.DONE;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return state == ScoreLevelThreeState.DONE;
    }
}