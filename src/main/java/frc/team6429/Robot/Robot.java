// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;

import frc.team6429.periodics.Auto.AutoModeExecutor;
import frc.team6429.periodics.Teleop.DriveTeleop;
import frc.team6429.periodics.Teleop.TeleopPeriodic;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Drivepanel;
import frc.team6429.subsystems.Gamepad;
import frc.team6429.subsystems.Indexer;
import frc.team6429.subsystems.LED;
import frc.team6429.util.Sensors;

import javax.management.loading.MLet;

import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  private Gamepad mGamepad;
  private Drivepanel mDrivepanel;
  private Drive mDrive;
  private Indexer mIntake;
  private Sensors mSensors;
  private DriveTeleop mDriveTeleop;
  private TeleopPeriodic mTeleopPeriodic;
  private AutoModeExecutor ame;
  private Timer timer;
  private LED mLED;


  
  /**
   *
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption();
    //m_chooser.addOption();
    //m_chooser.addOption();
    SmartDashboard.putData("Auto choices", m_chooser);
    mDrive = Drive.getInstance();
    mDriveTeleop = DriveTeleop.getInstance();
    mTeleopPeriodic = TeleopPeriodic.getInstance();
    mDrivepanel = Drivepanel.getInstance();
    mGamepad = Gamepad.getInstance();
    mIntake = Indexer.getInstance();
    mSensors = Sensors.getInstance();
    timer = new Timer();
    ame = new AutoModeExecutor();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("enc", mSensors.leftCANcoder.getPosition());
    SmartDashboard.putNumber("enc2", mSensors.rightCANcoder.getPosition());
    mSensors.pigeonOutput();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    ame.stop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mDriveTeleop.driveTeleop();
    mTeleopPeriodic.teleopPeriodic();
    /* Teleop: Robot drive
    double speed = mGamepad.getForward() - mGamepad.getReverse();
    double rotation;

    if (Math.abs(mGamepad.getSensetiveSteering()) > 0.2){
      rotation = mGamepad.getSensetiveSteering() * 0.5;
    }
    else{
      rotation = mGamepad.getSteering() * 0.75;
    }
    mDrive.robotDrive(speed, rotation);
    mGamepad.forceFeedback(speed, rotation);
  */
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    
    mSensors.resetCANcoder();
    mSensors.rightCANcoder.setPositionToAbsolute();
    mSensors.leftCANcoder.setPositionToAbsolute();
    mSensors.gyroReset();
    mLED = LED.getInstance();
    
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    mLED.setRainbow(1, 1, 20);

    //mDriveTeleop.driveTeleop();
    //mTeleopPeriodic.teleopPeriodic();


    /*if(mGamepad.getTest2()){
    mDrive.rightMotor.set(0.2);
    }

    else if(mGamepad.getTest1()){
      mDrive.leftMotor.set(0.2);
    }

    else{

      mDrive.stopDrive();
    } */
  /*mSensors.leftCANcoder.getMagnetFieldStrength();
  
  mDrive.rightMotor.set(1);
  mSensors.rightCANcoder.getPosition();
  mDrive.leftMotor.set(1);
  
  mSensors.leftCANcoder.getPosition();*/


    
    

  
  }

}
