package frc.robot.subsystems;

import java.net.CacheRequest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.WheelConstants;

//evil code below
public class MaxWheelModule {
    
    private CANSparkMax angleMotor, speedMotor;
    private SparkAbsoluteEncoder turningEncoder;


    private SparkPIDController speedPIDController;
    private PIDController anglePIDController;

    public MaxWheelModule(CANSparkMax angleMotor, CANSparkMax speedMotor){
        this.angleMotor = angleMotor;
        this.speedMotor = speedMotor;

        this.turningEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        this.speedPIDController = this.speedMotor.getPIDController();
        speedPIDController.setP(0.001);
        speedPIDController.setI(0);
        speedPIDController.setD(0);
        speedPIDController.setFF(0.25);

        this.speedMotor.getEncoder().setPosition(0);
        this.speedMotor.getEncoder().setPositionConversionFactor(WheelConstants.ROTATIONS_TO_METERS);
        this.speedMotor.getEncoder().setVelocityConversionFactor(this.speedMotor.getEncoder().getPositionConversionFactor()/60.0);

        this.anglePIDController = new PIDController(1.75, 0, 0);
        anglePIDController.enableContinuousInput(0, 1);
        anglePIDController.setTolerance(1.0/360);
    }
    
    public double getEncoderPosition(){
        return turningEncoder.getPosition();
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            speedMotor.getEncoder().getPosition(), Rotation2d.fromRotations(getEncoderPosition()));
	}

    public double getSpeed(){
        return speedMotor.getEncoder().getVelocity();
    }

    public void setSpeed(double speed){
        speedMotor.set(speed);
    }

    public SparkAbsoluteEncoder getTurningEncoder(){
        return turningEncoder;
    }
}
