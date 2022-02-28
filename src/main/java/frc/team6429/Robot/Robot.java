// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import frc.team6429.periodics.Auto.AutoModeExecutor;
import frc.team6429.periodics.Auto.Action.CreateTrajectoryAction.PathType;
import frc.team6429.periodics.Auto.AutoModes.MainAuto.FourCargoAuto;
import frc.team6429.periodics.Auto.AutoModes.MainAuto.ThreeCargoAuto;
import frc.team6429.periodics.Auto.AutoModes.MainAuto.TwoCargoAuto;
import frc.team6429.periodics.Auto.AutoModes.SimpleAuto.SimpleTwoCargo;
import frc.team6429.periodics.Teleop.DriveTeleop;
import frc.team6429.periodics.Teleop.TeleopPeriodic;
import frc.team6429.robot.RobotData.AnimationTypes;
import frc.team6429.subsystems.Drive;
import frc.team6429.subsystems.Drivepanel;
import frc.team6429.subsystems.Dumper;
import frc.team6429.subsystems.Gamepad;
import frc.team6429.subsystems.Climb;
import frc.team6429.subsystems.ClimbRemastered;
import frc.team6429.subsystems.Indexer;
import frc.team6429.subsystems.LED;
import frc.team6429.util.Sensors;
import frc.team6429.util.Utils;

import java.util.ArrayList;
import java.util.List;

import javax.management.loading.MLet;
import javax.swing.plaf.basic.BasicBorders.RadioButtonBorder;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kTwoCargoAuto = "Two Cargo";
  private static final String kThreeCargoAuto = "Three Cargo";
  private static final String kFourCargoAuto = "Four Cargo";
  private static final String kAlternateFourCargo = "Alternate Four Cargo";
  private static final String kFiveCargoAuto = "Five Cargo";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private AutoModeExecutor autoModeExecutor;
  private List<List<String>> listOfTrajectoryPaths;
  private double x = 0;
  private boolean led = true;
  private double rotation;
  private double speed;
  private double sensetiveSteering = 0.5;
  private double steering = 0.75;
  private double turnPID = 0.07;
  

  //systems
  private Sensors mSensors;
  private Drive mDrive;
  private Indexer mIndexer;
  private Dumper mDumper;
  private LED mLED;
  //private Climb mClimb;
  private ClimbRemastered mRemaster;
  private Gamepad mGamepad;
  private Drivepanel mDrivepanel;
  private DriveTeleop drivebaseTeleop;
  private TeleopPeriodic teleopPeriodic;
  private Timer timer;
  public double prevTime;

  private Thread thread = new Thread(new Runnable(){
    @Override
    public void run() {
      double startTime = Timer.getFPGATimestamp();
      DriverStation.reportWarning("Creating all trajectories in a thread", false);
      listOfTrajectoryPaths = new ArrayList<List<String>>();
      List<String> twoCargo = new ArrayList<>();
      List<String> threeCargo = new ArrayList<>();
      List<String> fourCargo = new ArrayList<>();
      twoCargo.addAll(List.of(
      "paths/twoCargo00.wpilib.json",
      "paths/twoCargo01.wpilib.json"));
      threeCargo.addAll(List.of(
      "paths/threeCargo00.wpilib.json",
      "paths/threeCargo01.wpilib.json",
      "paths/threeCargo02.wpilib.json",
      "paths/threeCargo03.wpilib.json"));
      fourCargo.addAll(List.of(
      "paths/fourCargo00v2.wpilib.json",
      "paths/fourCargo01v2.wpilib.json",
      "paths/fourCargo02v2.wpilib.json",
      "paths/fourCargo03v2.wpilib.json",
      "paths/fourCargo04v2.wpilib.json",
      "paths/fourCargo05v2.wpilib.json"));

      for(String trajectoryPath:twoCargo){
          RobotData.twoCargo.add(Utils.readTrajectoryFromPW(trajectoryPath));
        }
      for(String trajectoryPath:threeCargo){
          RobotData.threeCargo.add(Utils.readTrajectoryFromPW(trajectoryPath));
        }
      for(String trajectoryPath:fourCargo){
          RobotData.fourCargo.add(Utils.readTrajectoryFromPW(trajectoryPath));
        }
      
      DriverStation.reportWarning("Trajectories created in " + (Timer.getFPGATimestamp() - startTime) + "seconds", false);
    }                          
  }
); 
  /**
   *
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit(){
    m_chooser.setDefaultOption("Four Cargo Autonomous", kFourCargoAuto);
    m_chooser.addOption("Alternate Four Cargo", kAlternateFourCargo);
    m_chooser.addOption("Three Cargo Autonomous", kThreeCargoAuto);
    m_chooser.addOption("Two Cargo Autonomous", kTwoCargoAuto);
    m_chooser.addOption("Five Cargo Autonomous", kFiveCargoAuto);
    m_chooser.addOption("Default Autonomous", kDefaultAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    mSensors = Sensors.getInstance();
    mDrive = Drive.getInstance();
    mIndexer = Indexer.getInstance();
    //mClimb = Climb.getInstance();
    mRemaster = ClimbRemastered.getInstance();
    mDumper = Dumper.getInstance();
    mLED = LED.getInstance();
    mDrivepanel = Drivepanel.getInstance();
    mGamepad = Gamepad.getInstance();
    drivebaseTeleop = DriveTeleop.getInstance();
    teleopPeriodic = TeleopPeriodic.getInstance();
    autoModeExecutor = new AutoModeExecutor();
    timer = new Timer();
    timer.reset();
    timer.start();
    prevTime = timer.get();
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
    switch (m_autoSelected) {
      case kDefaultAuto:
        autoModeExecutor.setAutoMode(new FourCargoAuto(PathType.FOURCARGO));
        break;
      case kFourCargoAuto:
        autoModeExecutor.setAutoMode(new FourCargoAuto(PathType.FOURCARGO));
        break;
      case kThreeCargoAuto:
        autoModeExecutor.setAutoMode(new ThreeCargoAuto(PathType.THREECARGO));
        break;
      case kTwoCargoAuto:
        autoModeExecutor.setAutoMode(new TwoCargoAuto(PathType.TWOCARGO));
        break;
      default:
        autoModeExecutor.setAutoMode(new FourCargoAuto(PathType.FOURCARGO));
        break;
    }
    System.out.println("Auto selected: " + m_autoSelected);
    if (thread.isAlive()){
      DriverStation.reportWarning("Still not initialized the trajectories", false);
    }
    else{
      autoModeExecutor.start();
    }
    
    //mSensors.resetSensors();
    //mSensors.turnOnBothSensors();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*switch (m_autoSelected) {
      case kDefaultAuto:
        //
        break;
      case kTwoCargoAuto:
        // 
        break;
      case kThreeCargoAuto:
        //
        break;
      case kFourCargoAuto:
        //
        break;
    }*/
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    autoModeExecutor.stop();
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    speed = mGamepad.getForward() - mGamepad.getReverse();
    rotation = (mGamepad.getSteering()) * (steering);

    //Drive Shifter
    if(mGamepad.getDriveShiftOnePressed()) {
            mDrive.driveShiftOne();
    }

    else if(mGamepad.getDriveShiftTwoPressed()) {
            mDrive.driveShiftTwo();
    }
  
    //Test
    if(mGamepad.getTestPTO()){
            mDrive.powerTakeOff(!mDrive.pto.get());
    }

    if(mGamepad.getTestBrake()){
            mRemaster.brake(!mRemaster.frictionBrake.get());
    }

    mDrive.robotDrive(speed, rotation);
    mGamepad.forceFeedback(speed, rotation);

    //Indexer 
    if(mGamepad.getCustomIndexerOn()){
          //mIndexer.runWithBallCounter(1, 1);
          //mIndexer.pivotDown();

          if(mSensors.ballCounter() == 2){
              mLED.select(AnimationTypes.STROBEGREEN);
          }

          else if(mSensors.ballCounter() == 1){
              mLED.select(AnimationTypes.STROBEBLUE);
          }

          else{
              mLED.select(AnimationTypes.COLORFLOW);
          }
    }

    else if(mGamepad.getCustomIndexerReverse()){
               //mIndexer.indexerReverse(1, 1);
               //mIndexer.pivotDown();

              if(mSensors.ballCounter() == 2){
                  mLED.select(AnimationTypes.STROBEGREEN);
              }

              else if(mSensors.ballCounter() == 1){
                  mLED.select(AnimationTypes.STROBEBLUE);
              }

              else{
                  mLED.select(AnimationTypes.COLORFLOWOPP);
              }
    }
    
    else if(mGamepad.getDumperGamepad()){
              //mDumper.dumperSendWithIndexer(1, 1, 1);

              mLED.select(AnimationTypes.GREENFLOW);
    }

    else if(mGamepad.getDumperOppositeGamepad()){
              //mDumper.dumperOppositeWithIndexer(1, 1, 1);
              //mIndexer.pivotUp();

              mLED.select(AnimationTypes.GREENFLOWOPP);
    }

    else{
              mDumper.dumperStopWithIndexer();
              mIndexer.pivotUp();
              mLED.candle.setLEDs(0, 0, 0);
    }

}
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    
  }
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {


  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    mLED.candle.animate(new RainbowAnimation(1, 1, 68));
    //mIndexer.conveyorOn(1);
    //mDumper.dumperMotor.set(1);
    //SmartDashboard.putNumber("ball", mSensors.getBallCount());
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
