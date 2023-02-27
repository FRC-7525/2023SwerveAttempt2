package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

enum ArmStates {
    OFF,
    CUBE_ON,
    CONE_ON,
    TURNING_OFF,
    WAITING_FOR_FLOOR_INTAKE,
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
    CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax followMotor = new CANSparkMax(2, MotorType.kBrushless);
    Solenoid arm = new Solenoid(PneumaticsModuleType.REVPH, 0);
    PIDController controller = new PIDController(4, 0, 0);
    DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    ArmStates state = ArmStates.OFF;
    ArmStates nextState = ArmStates.OFF;
    Timer turningOffTimer = new Timer();
    Timer floorTimer = new Timer();
    Robot robot = null;
    double setpoint = 0.8;

    public Arm(Robot robot) {
        this.robot = robot;
        motor.restoreFactoryDefaults();
        followMotor.follow(motor, true);
    }

    private void toSetpoint() {
        if (encoder.getAbsolutePosition() != 0) {
            motor.set(controller.calculate(encoder.getAbsolutePosition(), setpoint));
        } else {
            motor.stopMotor();
            System.out.println("ARM ENCODER UNPLUGGED");
        }
    }

    public void periodic() {
        // Put encoder value on SmartDashboard
        SmartDashboard.putNumber("Arm Encoder Position", encoder.getAbsolutePosition());
        SmartDashboard.putBoolean("NearSetpoint", this.nearSetpoint());
        SmartDashboard.putBoolean("WaitingForFloorIntake", this.waitingForFloorIntake());

        if (state == ArmStates.OFF) {
            // Set position to low
            setpoint = 0.8;
            arm.set(false);
            stateString = "Off";
            robot.floorIntake.setState(FloorIntakeStates.OFF);
        } else if (state == ArmStates.CUBE_ON) {
            // Set position to high
            setpoint = 0.745;
            arm.set(false);
            stateString = "Intaking Cube";
            robot.floorIntake.setState(FloorIntakeStates.ON);
        } else if (state == ArmStates.CONE_ON) {
            setpoint = 0.6;
            stateString = "Intaking Cone";
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            if (nearSetpoint()) {
                arm.set(true);
            } else {
                arm.set(false);
            }
        } else if (state == ArmStates.TURNING_OFF) {
            turningOffTimer.start();
            arm.set(false);
            stateString = "Arm Resetting (for safety)";
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            if (turningOffTimer.get() > 3) {
                state = ArmStates.OFF;
                turningOffTimer.reset();
                turningOffTimer.stop();
            } else if (turningOffTimer.get() > 2) {
                setpoint = 0.8;
            }
        } else if (state == ArmStates.WAITING_FOR_FLOOR_INTAKE) {
            floorTimer.start();
            setpoint = 0.8;
            stateString = "Moving Floor Intake (Pre-Cone Intake)";
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            arm.set(false);

            if (floorTimer.get() > 0.8) {
                state = nextState;
                floorTimer.reset();
                floorTimer.stop();
            }
        } else if (state == ArmStates.LEVEL_ONE) {
            stateString = "Level One Scoring";
            setpoint = 0.69;
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
        } else if (state == ArmStates.LEVEL_TWO) {
            stateString = "Level Two Scoring";
            setpoint = 0.6;
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            if (nearSetpoint()) {
                arm.set(true);
            } else {
                arm.set(false);
            } 
        } else if (state == ArmStates.LEVEL_THREE) {
            stateString = "Level Three Scoring";
            setpoint = 0.6;
            robot.floorIntake.setState(FloorIntakeStates.DOWN_HOLD);
            if (nearSetpoint()) {
                arm.set(true);
            } else {
                arm.set(false);
            }
        }

        SmartDashboard.putString("Arm State", stateString);
        this.toSetpoint();
    }

    public boolean nearSetpoint() {
        return Math.abs(encoder.getAbsolutePosition() - setpoint) < 0.02;
    }

    public boolean waitingForFloorIntake() {
        return state == ArmStates.WAITING_FOR_FLOOR_INTAKE;
    }

    public void setState(ArmSetStates state) {
        if (this.state == ArmStates.OFF) {
            if (state == ArmSetStates.CONE_ON) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE;
                this.nextState = ArmStates.CONE_ON;
            }
            if (state == ArmSetStates.LEVEL_ONE) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE;
                this.nextState = ArmStates.LEVEL_ONE;
            }
            if (state == ArmSetStates.LEVEL_TWO) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE;
                this.nextState = ArmStates.LEVEL_TWO;
            }
            if (state == ArmSetStates.LEVEL_THREE) {
                floorTimer.reset();
                this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE;
                this.nextState = ArmStates.LEVEL_THREE;
            }
            if (state == ArmSetStates.CUBE_ON) this.state = ArmStates.CUBE_ON;
        } else if (this.state == ArmStates.CUBE_ON) {
            if (state == ArmSetStates.OFF) this.state = ArmStates.OFF;
            if (state == ArmSetStates.CONE_ON) this.state = ArmStates.CONE_ON;
        } else if (this.state == ArmStates.TURNING_OFF) {
            if (state == ArmSetStates.CONE_ON) this.state = ArmStates.CONE_ON;
            if (state == ArmSetStates.CUBE_ON) this.state = ArmStates.CUBE_ON;
        } else if (this.state == ArmStates.CONE_ON) {
            if (state == ArmSetStates.OFF || state == ArmSetStates.CUBE_ON) {
                turningOffTimer.reset();
                this.state = ArmStates.TURNING_OFF;
            }
        } else if (this.state == ArmStates.LEVEL_ONE) {
            if (state == ArmSetStates.OFF) {
                if (robot.intake.isCone()) {
                    this.state = ArmStates.WAITING_FOR_FLOOR_INTAKE;
                    this.nextState = ArmStates.OFF;
                } else {
                    this.state = ArmStates.OFF;
                }
            }
        } else if (this.state == ArmStates.LEVEL_TWO || this.state == ArmStates.LEVEL_THREE) {
            if (state == ArmSetStates.OFF) {
                turningOffTimer.reset();
                this.state = ArmStates.TURNING_OFF;
            }
        }
    }
}
