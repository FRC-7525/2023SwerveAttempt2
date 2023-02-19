package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

enum ArmStates {
    OFF,
    CUBE_ON,
    CONE_ON,
    TURNING_OFF
}

enum ArmSetStates {
    OFF,
    CUBE_ON,
    CONE_ON
}

public class Arm {
    CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
    Solenoid arm = new Solenoid(PneumaticsModuleType.REVPH, 0);
    PIDController controller = new PIDController(4, 0, 0);
    DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    ArmStates state = ArmStates.OFF;
    Timer timer = new Timer();
    Robot robot = null;
    double setpoint = 0.72;

    public Arm(Robot robot) {
        this.robot = robot;
        motor.restoreFactoryDefaults();
    }

    private void toSetpoint() {
        motor.set(controller.calculate(encoder.getAbsolutePosition(), setpoint));
    }

    public void periodic() {
        // Put encoder value on SmartDashboard
        SmartDashboard.putNumber("Arm Encoder Position", encoder.getAbsolutePosition());

        if (state == ArmStates.OFF) {
            // Set position to low
            setpoint = 0.8;
            arm.set(false);
        } else if (state == ArmStates.CUBE_ON) {
            // Set position to high
            setpoint = 0.7;
            arm.set(false);
        } else if (state == ArmStates.CONE_ON) {
            arm.set(true);
            setpoint = 0.62;
        } else if (state == ArmStates.TURNING_OFF) {
            setpoint = 0.75;
            arm.set(false);
            if (timer.get() > 2) {
                state = ArmStates.OFF;
            }
        }

        this.toSetpoint();
    }

    public void setState(ArmSetStates state) {
        if (this.state == ArmStates.OFF || this.state == ArmStates.CUBE_ON) {
            if (state == ArmSetStates.OFF) this.state = ArmStates.OFF;
            if (state == ArmSetStates.CONE_ON) this.state = ArmStates.CONE_ON;
            if (state == ArmSetStates.CUBE_ON) this.state = ArmStates.CUBE_ON;
        } else if (this.state == ArmStates.TURNING_OFF) {
            if (state == ArmSetStates.CONE_ON) this.state = ArmStates.CONE_ON;
            if (state == ArmSetStates.CUBE_ON) this.state = ArmStates.CUBE_ON;
        } else if (this.state == ArmStates.CONE_ON) {
            if (state == ArmSetStates.OFF) this.state = ArmStates.TURNING_OFF;
            if (state == ArmSetStates.CUBE_ON) this.state = ArmStates.TURNING_OFF;
        }
    }
}
