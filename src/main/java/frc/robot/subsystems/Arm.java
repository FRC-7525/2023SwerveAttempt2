package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

enum States {
    OFF,
    ON
}

public class Arm {
    CANSparkMax motor = new CANSparkMax(8, MotorType.kBrushless);
    SparkMaxPIDController motorPIDcontroller = motor.getPIDController();
    RelativeEncoder motorEncoder = motor.getEncoder();
    States state = States.OFF;
    Robot robot = null;

    public Arm(Robot robot) {
        this.robot = robot;
        
        motor.restoreFactoryDefaults();

        motorPIDcontroller.setP(0.001);
        motorPIDcontroller.setI(0);
        motorPIDcontroller.setD(0);
        motorPIDcontroller.setIZone(0);
        motorPIDcontroller.setFF(0.001);
        motorPIDcontroller.setOutputRange(-0.2, 0.2);

        motorPIDcontroller.setSmartMotionMaxVelocity(2000, 0);
        motorPIDcontroller.setSmartMotionMinOutputVelocity(0, 0);
        motorPIDcontroller.setSmartMotionMaxAccel(1500, 0);
        motorPIDcontroller.setSmartMotionAllowedClosedLoopError(100, 0);
    }

    public void periodic() {
        // Put encoder value on SmartDashboard
        SmartDashboard.putNumber("Arm Encoder Position", motorEncoder.getPosition());
        SmartDashboard.putNumber("Arm Encoder Velocity", motorEncoder.getVelocity());

        if (state == States.OFF) {
            // Set position to 0
            motorPIDcontroller.setReference(0, CANSparkMax.ControlType.kSmartMotion);

            if (robot.secondaryController.getLeftBumperPressed()) {
                state = States.ON;
            }
        } else if (state == States.ON) {
            // Set position to 10000
            motorPIDcontroller.setReference(1000, CANSparkMax.ControlType.kSmartMotion);

            if (robot.secondaryController.getLeftBumperPressed()) {
                state = States.OFF; 
            }
        }
    }
}
