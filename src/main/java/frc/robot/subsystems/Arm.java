package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

enum ArmStates {
    OFF,
    ON
}

public class Arm {
    CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
    SparkMaxPIDController motorPIDcontroller = motor.getPIDController();
    RelativeEncoder motorEncoder = motor.getEncoder();
    ArmStates state = ArmStates.OFF;
    DutyCycleEncoder DCencoder = new DutyCycleEncoder(0);
    Robot robot = null;

    public Arm(Robot robot) {
        this.robot = robot;
        
        motor.restoreFactoryDefaults();

        motorPIDcontroller.setP(0.001);
        motorPIDcontroller.setI(0);
        motorPIDcontroller.setD(0);
        motorPIDcontroller.setIZone(0);
        motorPIDcontroller.setFF(0.001);
        motorPIDcontroller.setOutputRange(0.0, 0.0);

        motorPIDcontroller.setSmartMotionMaxVelocity(2000, 0);
        motorPIDcontroller.setSmartMotionMinOutputVelocity(0, 0);
        motorPIDcontroller.setSmartMotionMaxAccel(1500, 0);
        motorPIDcontroller.setSmartMotionAllowedClosedLoopError(100, 0);
    }

    public void periodic() {
        // Put encoder value on SmartDashboard
        SmartDashboard.putNumber("Arm Encoder Position", DCencoder.getAbsolutePosition());
        //SmartDashboard.putNumber("Arm Encoder Velocity", encoder.geT());

        if (state == ArmStates.OFF) {
            // Set position to 0
            motorPIDcontroller.setReference(0, CANSparkMax.ControlType.kSmartMotion);

            if (robot.primaryController.getRightBumperPressed()) {
                state = ArmStates.ON;
            }
        } else if (state == ArmStates.ON) {
            // Set position to 10000
            motorPIDcontroller.setReference(10000, CANSparkMax.ControlType.kSmartMotion);
 
            if (robot.primaryController.getRightBumperPressed()) {
                state = ArmStates.OFF; 
            }
        }
    }
}
