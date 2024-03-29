package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

enum ArmStates {
    OFF,
    CUBE_ON,
    CONE_ON,
    TURNING_OFF,
    WAITING_FOR_FLOOR_INTAKE_DOWN,
    WAITING_FOR_FLOOR_INTAKE_UP,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE
}

enum ArmSetStates {
    OFF,
    CUBE_ON,
    CONE_ON,
    LEVEL_ONE,
    LEVEL_TWO,
    LEVEL_THREE
}

public class Arm {
    private String stateString;
    CANSparkMax pivotMotor = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax pivotFollower = new CANSparkMax(2, MotorType.kBrushless);
    PIDController pivotController = new PIDController(5, 0, 0);
    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
    final double DOWN = 0.36;
    double pivotSetpoint = DOWN;

    CANSparkMax extensionMotor = new CANSparkMax(11, MotorType.kBrushless);
    PIDController extensionController = new PIDController(0.1, 0, 0);
    RelativeEncoder extensionEncoder = extensionMotor.getEncoder();

    ArmStates state = ArmStates.OFF;
    ArmStates nextState = ArmStates.OFF;
    Timer turningOffTimer = new Timer();
    Timer floorTimer = new Timer();
    Robot robot = null;
    final double ARM_OUT = 23.5;
    double extensionSetpoint = 0;

    final double CUBE_INTAKE = 0.045;

    StringLogEntry armStateLog;

    public Arm(Robot robot) {
        this.robot = robot;
        pivotMotor.restoreFactoryDefaults();
        pivotFollower.follow(pivotMotor, true);

        extensionMotor.restoreFactoryDefaults();
        extensionEncoder.setPosition(0);

        DataLog log = DataLogManager.getLog();
        armStateLog = new StringLogEntry(log, "/arm/state");
    }

    private void toSetpoint() {
        if (pivotEncoder.getAbsolutePosition() > 0.1) {
            pivotMotor.set(pivotController.calculate(pivotEncoder.getAbsolutePosition(), pivotSetpoint));
        } else {
            pivotMotor.stopMotor();
            System.out.println("ARM ENCODER UNPLUGGED");
        }

        extensionMotor.set(extensionController.calculate(extensionEncoder.getPosition(), extensionSetpoint));
    }

    public void reset() {
        state = ArmStates.OFF;
    }

    public void putEncoderPosition() {
        SmartDashboard.putNumber("Pivot Encoder Position", pivotEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Extender Encoder Position", extensionEncoder.getPosition());
        SmartDashboard.putBoolean("Extender Near Zero?", Math.abs(extensionEncoder.getPosition()) < 0.5);
    }

    public void periodic() {
        // Put encoder value on SmartDashboard
        SmartDashboard.putBoolean("NearSetpoint", this.nearSetpoint());
        SmartDashboard.putBoolean("WaitingForFloorIntake", this.waitingForFloorIntake());

        if (state == ArmStates.OFF) {
            // Set position to low
            pivotSetpoint = DOWN;
            extensionSetpoint = 0;
            stateString = "Off";
            robot.floorIntake.setState(FloorIntakeStates.OFF);
        } else if (state == ArmStates.CUBE_ON) {
            // Set position to high
            pivotSetpoint = DOWN - CUBE_INTAKE;
            extensionSetpoint = 0;
            stateString = "Intaking Cube";
            robot.floorIntake.setState(FloorIntakeStates.ON);
        } else if (state == ArmStates.CONE_ON) {
            pivotSetpoint = DOWN - 0.195;
            stateString = "Intaking Cone";
            if (nearSetpoint()) {
                robot.floorIntake.setState(FloorIntakeStates.OFF);
            } else {
                robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            }
        } else if (state == ArmStates.WAITING_FOR_FLOOR_INTAKE_UP) {
            floorTimer.start();
            pivotSetpoint = DOWN - 0.195;
            stateString = "Moving Floor Intake Out (Pre-Arm Down)";
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            extensionSetpoint = 0;

            if (floorTimer.get() > 0.8) {
                state = ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
                nextState = ArmStates.OFF;
                floorTimer.reset();
                floorTimer.stop();
            }
        } else if (state == ArmStates.TURNING_OFF) {
            turningOffTimer.start();
            extensionSetpoint = 0;
            stateString = "Arm Resetting (for safety)";
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            if (turningOffTimer.get() > 1.1) {
                state = ArmStates.OFF;
                turningOffTimer.reset();
                turningOffTimer.stop();
            } else if (turningOffTimer.get() > 0.3) {
                pivotSetpoint = DOWN;
            }
        } else if (state == ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN) {
            floorTimer.start();
            pivotSetpoint = DOWN;
            stateString = "Moving Floor Intake Pre-Arm Up / Move Arm Down";
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            extensionSetpoint = 0;

            if (floorTimer.get() > 0.8) {
                state = nextState;
                floorTimer.reset();
                floorTimer.stop();
            }
        } else if (state == ArmStates.LEVEL_ONE) {
            stateString = "Level One Scoring";
            if (robot.intake.isCone()) {
                pivotSetpoint = DOWN - 0.11;
                robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            } else {
                pivotSetpoint = DOWN - CUBE_INTAKE;
                robot.floorIntake.setState(FloorIntakeStates.OUTTAKE);
            }
        } else if (state == ArmStates.LEVEL_TWO) {
            stateString = "Level Two Scoring";
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            if (robot.intake.isCone()) {
                pivotSetpoint = DOWN - 0.205;
                if (nearSetpoint()) {
                    extensionSetpoint = ARM_OUT;
                } else {
                    extensionSetpoint = 0;
                } 
            } else {
                pivotSetpoint = DOWN - 0.18;
                extensionSetpoint = 0;
            }
        } else if (state == ArmStates.LEVEL_THREE) {
            stateString = "Level Three Scoring";
            pivotSetpoint = DOWN - 0.22;
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            if (nearSetpoint()) {
                extensionSetpoint = ARM_OUT;
            } else {
                extensionSetpoint = 0;
            }
        }

        SmartDashboard.putString("Arm State", stateString);
        this.toSetpoint();
        armStateLog.append(stateString);
    }

    public boolean nearSetpoint() {
        return Math.abs(pivotEncoder.getAbsolutePosition() - pivotSetpoint) < 0.02;
    }

    public boolean waitingForFloorIntake() {
        return state == ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
    }

    public void setState(ArmSetStates state) {
        if (this.state == ArmStates.OFF) {
            if (state == ArmSetStates.CONE_ON) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
                this.nextState = ArmStates.CONE_ON;
            }
            if (state == ArmSetStates.LEVEL_ONE) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
                this.nextState = ArmStates.LEVEL_ONE;
            }
            if (state == ArmSetStates.LEVEL_TWO) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
                this.nextState = ArmStates.LEVEL_TWO;
            }
            if (state == ArmSetStates.LEVEL_THREE) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
                this.nextState = ArmStates.LEVEL_THREE;
            }
            if (state == ArmSetStates.CUBE_ON) this.state = ArmStates.CUBE_ON;
        } else if (this.state == ArmStates.CUBE_ON) {
            if (state == ArmSetStates.OFF) this.state = ArmStates.OFF;
            if (state == ArmSetStates.CONE_ON) this.state = ArmStates.CONE_ON;
        } else if (this.state == ArmStates.CONE_ON) {
            if (state == ArmSetStates.OFF || state == ArmSetStates.CUBE_ON) {
                turningOffTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE_UP;
            }
        } else if (this.state == ArmStates.LEVEL_ONE) {
            if (state == ArmSetStates.OFF) {
                if (robot.intake.isCone()) {
                    floorTimer.reset();
                    this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
                    this.nextState = ArmStates.OFF;
                } else {
                    this.state = ArmStates.OFF;
                }
            }
        } else if (this.state == ArmStates.LEVEL_TWO) {
            if (state == ArmSetStates.OFF) {
                if (robot.intake.isCone()) {
                    turningOffTimer.reset();
                    this.state = ArmStates.TURNING_OFF;
                } else {
                    floorTimer.reset();
                    this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE_DOWN;
                    this.nextState = ArmStates.OFF;
                }
            }
        } else if (this.state == ArmStates.LEVEL_THREE) {
            if (state == ArmSetStates.OFF) {
                turningOffTimer.reset();
                this.state = ArmStates.TURNING_OFF;
            }
        }
    }

}
