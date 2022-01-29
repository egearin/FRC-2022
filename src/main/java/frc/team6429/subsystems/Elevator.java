package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.R4D9.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Timer;

public class Elevator {

    private static Elevator mInstance = new Elevator();

    public static Elevator getInstance(){
        return mInstance;
    }

    public double elevator1RPM;
    public double elevator2RPM;
    public double kP,kI,kD;//for PID
    public double kS,kG,kV,kA;
    //for feedforward, kS and kG should be "volt"; kV should be "volts*seconds/distance"; kA is acceleration, may be omitted

    public double highMax;
    public double lowMax;
    public double lowClamp;
    public double highClamp;
    public double outputVoltage;

    public PIDController upwardsPIDController;
    public PIDController downwardsPIDController;
    public PIDController elevatorPID;

    public ElevatorFeedforward elevatorUpwardsFF;
    public ElevatorFeedforward elevatorDownwardsFF;
    public ElevatorFeedforward elevatorFF;

    public VictorSP motor1;
    public Encoder encoder1;

    public Timer timer;
    public Timer innerTimer;

    private Elevator(){

        timer = new Timer();
        innerTimer = new Timer();
        upwardsPIDController = new PIDController(Constants.kUpwardsElevatorP, Constants.kUpwardsElevatorI, Constants.kUpwardsElevatorD);
        downwardsPIDController = new PIDController(Constants.kDownwardsElevatorP, Constants.kDownwardsElevatorI, Constants.kDownwardsElevatorI);
        elevatorPID = new PIDController(kP,kI,kD);
        elevatorUpwardsFF = new ElevatorFeedforward(Constants.kUpwardsElevatorS, Constants.kUpwardsElevatorV, Constants.kUpwardsElevatorA, Constants.kUpwardsElevatorG);
        elevatorDownwardsFF = new ElevatorFeedforward(Constants.kDownwardsElevatorS, Constants.kDownwardsElevatorV, Constants.kDownwardsElevatorA, Constants.kDownwardsElevatorG);
        elevatorFF = new ElevatorFeedforward(kS,kG,kV);
        motor1 = new VictorSP(Constants.elevator1MotorPort);
        encoder1 = new Encoder(Constants.elevatorEncPort1, Constants.elevatorEncPort2);
        innerTimer.reset();
        timer.reset();
        timer.start();

    }

    public void resetEncoder(){
        encoder1.reset();
    }

    public void resetPID(){
        upwardsPIDController.reset();
        downwardsPIDController.reset();
    }

    public void resetEverything(){
        timer.reset();
        encoder1.reset();
        upwardsPIDController.reset();
        downwardsPIDController.reset();
    }

    public void setPIDCoefficients(double kDP, double kDI, double kDD, double kUP, double kUI, double kUD){
        upwardsPIDController.setPID(kUP, kUI, kUD);
        downwardsPIDController.setPID(kDP, kDI, kDD);
        SmartDashboard.putNumber("DOWNWARDS_PROPORTIONAL_COEFFICIENT:", kDP);
        SmartDashboard.putNumber("DOWNWARDS_INTEGRAL_COEFFICIENT:", kDI);
        SmartDashboard.putNumber("DOWNWARDS_DERIVATIVE_COEFFICIENT:", kDD);
        SmartDashboard.putNumber("UPWARDS_PROPORTIONAL_COEFFICIENT:", kUP);
        SmartDashboard.putNumber("UPWARDS_INTEGRAL_COEFFICIENT:", kUI);
        SmartDashboard.putNumber("UPWARDS_DERIVATIVE_COEFFICIENT:", kUD);
    }

    public void getPIDCoefficients(){
        SmartDashboard.getNumber("DOWNWARDS_PROPORTIONAL_COEFFICIENT:", Constants.kDownwardsElevatorP);
        SmartDashboard.getNumber("DOWNWARDS_INTEGRAL_COEFFICIENT:", Constants.kDownwardsElevatorI);
        SmartDashboard.getNumber("DOWNWARDS_DERIVATIVE_COEFFICIENT:", Constants.kDownwardsElevatorD);
        SmartDashboard.getNumber("UPWARDS_PROPORTIONAL_COEFFICIENT:", Constants.kUpwardsElevatorP);
        SmartDashboard.getNumber("UPWARDS_PROPORTIONAL_COEFFICIENT:", Constants.kUpwardsElevatorI);
        SmartDashboard.getNumber("UPWARDS_PROPORTIONAL_COEFFICIENT:", Constants.kUpwardsElevatorD);
    }

    public void setPIDTolerance(double errorTolerance, double derivativeTolerance){
        upwardsPIDController.setTolerance(errorTolerance, derivativeTolerance);
        SmartDashboard.putNumber("ERROR_TOLERANCE:", errorTolerance );
        SmartDashboard.putNumber("DERIVATIVE_TOLERANCE:", derivativeTolerance);
    }

    public void setPIDIntegratorRange(double minimumIntegral, double maximumIntegral){
        downwardsPIDController.setIntegratorRange(minimumIntegral, maximumIntegral);
        SmartDashboard.putNumber("MINIMUM_INTEGRAL:", minimumIntegral);
        SmartDashboard.putNumber("MAXIMUM_INTEGRAL:", maximumIntegral);
    
    }
    //in order to clamp the output of the PID control, handy for systems with limited movement
    public void setPIDClamp(double low_Clamp, double high_Clamp){
        lowClamp = low_Clamp;
        highClamp = high_Clamp;
        SmartDashboard.putNumber("LOW_CLAMP:", lowClamp);
        SmartDashboard.putNumber("HIGH_CLAMP:", highClamp);
    }

    /*public void setFFCoefficients(double maxSpeed, double minSpeed){
        elevatorFF.maxAchievableVelocity(Constants.maxVoltageInput, acceleration)
    }
    */

    public void ElevatorRun(double wantedState, double wantedVelocity, double currentState){

    /*
    if(encoder1.getRate() != 0){
        System.out.println("Elevator is not stable");
    }
    */

    currentState = encoder1.getDistance();

    outputVoltage = elevatorPID.calculate(currentState, wantedState);
    outputVoltage =+ elevatorFF.calculate(wantedState, wantedVelocity);

    /*if(wantedVector == true){
        motor1.setVoltage(outputVoltage);
    }

    if(wantedVector == false){
        motor1.setVoltage(-outputVoltage);
    }

    }*/

    motor1.setVoltage(outputVoltage);
    }

    /**
     * 
     * @param motionVector
     * @param motorVoltage
     * @param time
     */
    public void timerRun(boolean motionVector, double motorVoltage, double time){
        if(motionVector == true){//true if upwards
            innerTimer.start();
            motor1.setVoltage(motorVoltage);
        if(!timer.hasElapsed(time)){
            motor1.setVoltage(0.0);
            motor1.stopMotor();
            innerTimer.stop();
            innerTimer.reset();
            }
        }
        else if(motionVector == false){//false if downwards
            innerTimer.start();
        if(!timer.hasElapsed(time)){
            motor1.setVoltage(0.0);
            motor1.stopMotor();
            innerTimer.stop();
            innerTimer.reset();
            }
        }
    }


}