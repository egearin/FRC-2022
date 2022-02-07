package frc.team6429.subsystems;

// Default
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// Motor
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team6429.robot.Constants;
import frc.team6429.util.Utils;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// Sensor
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;

//PID Controller


// encoder
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

// Other 
import java.lang.Math;


public class Drive2 {

    private static Drive2 mInstance = new Drive2();

    public static Drive2 getInstance(){
        return mInstance;
    }

    private PIDController PID;
    //TalonFX
    private WPI_TalonFX leftOne;
    private WPI_TalonFX leftTwo;

    private WPI_TalonFX rightOne;
    private WPI_TalonFX rightTwo;

    //Motor Groups
    private MotorControllerGroup leftMotor;
    private MotorControllerGroup rightMotor;

    //Drivetrain
    private DifferentialDrive chassis;
    
    //Sensors
    PigeonIMU pigeon;
    private CANCoder driveLeftCANcoder;
    private CANCoder driveRightCANcoder;

    public Drive2(){
        leftOne = Utils.makeTalonFX(Constants.leftOneMotorID,false);
        leftTwo = Utils.makeTalonFX(Constants.leftTwoMotorID,false);

        rightOne = Utils.makeTalonFX(Constants.rightOneMotorID,true);
        rightTwo = Utils.makeTalonFX(Constants.rightTwoMotorID,true);

        leftMotor = new MotorControllerGroup(leftOne, leftTwo);
        leftMotor.setInverted(false);

        rightMotor = new MotorControllerGroup(rightOne, rightTwo);
        rightMotor.setInverted(true);
        
        chassis = new DifferentialDrive(leftMotor, rightMotor);

        pigeon = new PigeonIMU(Constants.pigeonID);

        driveLeftCANcoder = new CANCoder(Constants.leftCANcoderID);
        driveRightCANcoder = new CANCoder(Constants.rightCANcoderID);

        //rightOne.follow(rightTwo);
        //leftOne.follow(leftTwo);

        //leftTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        //rightTwo.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        /*
        leftTwo.setNeutralMode(NeutralMode.Brake);
        rightTwo.setNeutralMode(NeutralMode.Brake);
        leftOne.setNeutralMode(NeutralMode.Brake);
        rightOne.setNeutralMode(NeutralMode.Brake);*/
    }
    
      
    /**
     * Drive Outputs Using Smartdashboard
     */
    public void driveOutput(){

    }
    /**
     * Robot Drive Using Arcade Drive
     * @param speed
     * @param rotation_speed
     */
    public void robotDrive(double speed, double rotation_speed){
        robotDrive(speed, rotation_speed, 1);
    }

    /**
     * Robot Drive Using Arcade Drive
     * @param speed
     * @param rotation_speed
     * @param sensitivity
     */
    public void robotDrive(double speed, double rotation_speed, double sensivity){
        chassis.arcadeDrive(speed, rotation_speed*sensivity);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn){
        chassis.curvatureDrive(speed, rotation, isQuickTurn);
    }
    
    /**
     * 
     * Gets Gyro Angle
     */
    public double getGyroAngle(){
        return pigeon.getFusedHeading();
    }

    /**
     * Resets Gyro Values
     */
    public void gyroReset(){
        pigeon.setYaw(0);
        pigeon.setAccumZAngle(0);
        pigeon.setFusedHeading(0);
    }



    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(-rightVolts);
        chassis.feed();
    }

    /**
     * Fully automated drive with PID
     * @param speed
     * @param rotation_angle
     */
    public void PIDDrive(double speed, double rotation_angle){
        double drivePID = PID.calculate(getGyroAngle(), rotation_angle);
        drivePID = MathUtil.clamp(drivePID, -1, 1);
        chassis.curvatureDrive(speed, drivePID, false);
    }

    public void resetPID(){
        PID.reset();
    }

    /**
     * Resets everything resettable
     */
    public void reset(){
        resetPID();
        gyroReset();
        
    }

    public double simpleTurnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.01);
        double minMax = SmartDashboard.getNumber("Min PID", 0.3);
        if (desired_rotation > 1.0) {
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0) {
                rotation = kP*desired_rotation - minMax;
        }
        return rotation;
    }

    public double turnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.1);
        double minMax = 1.9;
        if (desired_rotation > 1.0) { // to the right
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0) { // to the left
                rotation = kP*desired_rotation - minMax;
        }
        tankDriveVolts(rotation, -rotation);
        return rotation;
    }
    /**
     * Stop Driving Robot
     */
    public void stopDrive(){
        chassis.tankDrive(0, 0);
    }
}
    