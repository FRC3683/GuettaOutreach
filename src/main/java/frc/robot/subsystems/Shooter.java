package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.config.Config;
import frc.robot.config.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NPointRemap;
import frc.robot.vision.Eye;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter(Robot.m_cfg);
        return instance;
    }

    private int readyCounter;
    private boolean readyFlag;
    
    private TalonSRX motor1;
    private VictorSPX motor2;

    private Solenoid hoodPiston;

    private double targetDistance;
    private boolean foundDistance;
    private double targetRadPerSec;
    private double output;
    private double previousRadPerSec;

    private double recentDistance;
    private double recentRadPerSec;
    
    private NPointRemap m_nPointRemap;
    
    Eye m_eye;
    
    private PIDController PID;
    private SimpleMotorFeedforward feedForward;
    
    private State currentState;
    
    public enum State {
        MANUAL_DISTANCE("MANUAL_DISTANCE"),
        DISABLED("DISABLED"),
        STOWED("STOWED"), 
        FINDING_DISTANCE("FINDING_DISTANCE"),
        REVING_NEAR("REVING_NEAR"), 
        SHOOTING_NEAR("SHOOTING_NEAR"), 
        REVING_FAR("REVING_FAR"),
        SHOOTING_FAR("SHOOTING_FAR");

        private String name;

        private State(String nString) {
            name = nString;
        }

        public String getName() {
            return name;
        }
    }

    public void setState(State state) {
        currentState = state;
    }
    
    public State getState() {
        return currentState;
    }

    public void setStateReving() {
        
            setState(State.MANUAL_DISTANCE);
        
        
    }

    public void setStateShooting() {
        
            setState(State.SHOOTING_NEAR);
    
        }
    

    private ShuffleboardTab tab;
    private NetworkTableEntry currentFlywheelSpeedEntry;
    private NetworkTableEntry targetFlywheelSpeedEntry;
    private NetworkTableEntry readyToFireEntry;
    private NetworkTableEntry stateEntry;
    private NetworkTableEntry ticksEntry;
    private NetworkTableEntry targetVEntry;
    private NetworkTableEntry readRadPerSec;
    private NetworkTableEntry limelightDistance;
    private NetworkTableEntry sensorValue;
    private NetworkTableEntry targetRPM;
    private NetworkTableEntry outputRPM;
    private NetworkTableEntry outputTicksPer100ms;

    private Shooter(Config cfg) {
        motor1 = cfg.getShooterMotor1();
        motor2 = cfg.getShooterMotor2();

        hoodPiston = cfg.getShooterHoodPiston();

        targetDistance = 0.0;
        targetRadPerSec = 0.0;
        foundDistance = false;
        output = 0.0;        
        previousRadPerSec = 0.0;
        currentState = State.DISABLED;
        readyCounter = 0;

        //PID = new PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD);
        motor1.config_kP(1, Constants.shooterP, 10);
        motor1.config_kI(1, Constants.shooterI, 10);
        motor1.config_kD(1, Constants.shooterD, 10);
        motor1.config_kF(1, Constants.shooterV, 10);
        motor1.config_IntegralZone(1, Constants.shooterIZone, 10);

        readyFlag = false;

        feedForward = new SimpleMotorFeedforward(Constants.shooterS, Constants.shooterV, Constants.shooterA);

        m_nPointRemap = new NPointRemap(Constants.shooterFile);
        //TODO add values
        
        m_eye = Eye.getInstance();

        // tab = Shuffleboard.getTab("Shooter");
        // //tab.add("Encoder", flywheelEnc);
        // targetVEntry = tab.add("target Voltage", 0).getEntry();
        // targetFlywheelSpeedEntry = tab.add("motor voltage", 0).getEntry();
        // readyToFireEntry = tab.add("readyToFire", 0).getEntry();
        // stateEntry = tab.add("state", currentState.getName()).getEntry();
        // ticksEntry = tab.add("ticks", getCurrentRadPerSec()).getEntry();
        // readRadPerSec = tab.add("Speed Set", 1).getEntry();
        // limelightDistance = tab.add("limelightDitance", m_eye.getDistanceFromTarget()).getEntry();
        // sensorValue = tab.add("sensor value", motor1.getSelectedSensorVelocity()).getEntry();
        // targetRPM = tab.add("targetRPM", targetRadPerSec*60/(2*Math.PI)).getEntry();
        // outputRPM = tab.add("outputRPM", ticksEntry.getDouble(0)*60/(2*Math.PI)).getEntry();
    }

    public void addRecentPoint() {
        m_nPointRemap.addPoint(recentDistance, recentRadPerSec);
        m_nPointRemap.compile();
    }


    private void handleStates() {
        switch (currentState) {

        case MANUAL_DISTANCE:
            handleMANUAL_DISTANCE();
        case DISABLED:
            handleDISABLED();
            break;
        case STOWED:
            handleSTOWED();
            break;
        case FINDING_DISTANCE:
            handleFINDING_DISTANCE();
            break;
        case REVING_NEAR:
            handleREVING_NEAR();
            break;
        case SHOOTING_NEAR:
            handleSHOOTING_NEAR();
            break;
        case REVING_FAR:
            handleREVING_FAR();
            break;
        case SHOOTING_FAR:
            handleSHOOTING_FAR();
            break;
        }
    }

    public boolean inNearShootingRange(){
        if(foundDistance){
            return targetDistance >= Constants.shooterNearMinDistance && 
                   targetDistance <= Constants.shooterNearMaxDistance;
        }
        return false;
    }

    public boolean inFarShootingRange(){
        if(foundDistance){
            return targetDistance >= Constants.shooterFarMinDistance && 
                   targetDistance <= Constants.shooterFarMaxDistance;
        }
        return false;
    }

    public boolean inShootingRange(){
        return inFarShootingRange() || inNearShootingRange();
    }

    private void hoodNear() {
        hoodPiston.set(false);
    }

    private void hoodFar() {
        hoodPiston.set(false);
    }

    private double getRadPerSecFromDistance(double distance) {
        return m_nPointRemap.calcY(distance);
    }

    private double calcOutput(double targetRadPerSec) {
        double targetTicksPer100ms = targetRadPerSec * Constants.shooterPulsePerRad / Constants.shooter100msPerS;
        return targetTicksPer100ms; //PID.calculate(getCurrentRadPerSec()) + feedForward.calculate(targetRadPerSec)/Constants.shooterMaxVoltage;
    }
    
    private void handleMANUAL_DISTANCE(){
        hoodNear();
        setTargetRadPerSec(150);
        output = calcOutput(targetRadPerSec);
        foundDistance = false;
    }

    private void handleDISABLED() {
        hoodNear();
        setTargetRadPerSec(0);
        foundDistance = false;
        output = 0;
    }

    private void handleSTOWED() {
        hoodNear();
        setTargetRadPerSec(0);
        foundDistance = false;
        output = 0;
        readyFlag = false;
    }

    private void handleFINDING_DISTANCE() {
        hoodNear();
        setTargetRadPerSec(0);

        if(m_eye.validTarget()) {
            targetDistance = m_eye.getDistanceFromTarget();
            foundDistance = true;
        } else {
            foundDistance = false;
        }
    }

    public boolean foundDistance() {
        return foundDistance;
    }

    private void handleREVING_NEAR() {
        hoodNear();
        setTargetRadPerSec(getRadPerSecFromDistance(targetDistance));
        output = calcOutput(targetRadPerSec);
    }

    private void handleSHOOTING_NEAR() {
        hoodNear();
        setTargetRadPerSec(150);
        output = calcOutput(targetRadPerSec);
        recentDistance = targetDistance;
        recentRadPerSec = targetRadPerSec;
    }

    private void handleREVING_FAR() {
        hoodFar();
        if(MathUtils.closeEnough(getCurrentRadPerSec(), targetRadPerSec, 15)){//&& inShootingRange() && Robot.m_eye.onTarget();){
            readyCounter++;
        }
        else{
            readyCounter=0;
        }
        if(readyCounter >=5){
            readyFlag = true;  
        }
        //setTargetRadPerSec(Constants.testSpeed);
        output = calcOutput(targetRadPerSec);
        //output = targetRadPerSec/1000;
    }
    
    private void handleSHOOTING_FAR() {
        hoodFar();
        //setTargetRadPerSec(getRadPerSecFromDistance(targetDistance));
        output = calcOutput(targetRadPerSec);
        //recentDistance = targetDistance;
        //recentRadPerSec = targetRadPerSec;
    }
    
    public void setOutput(double output){
        this.output=output;
    }

    public void setTargetRadPerSec(double radPerSec) {
        targetRadPerSec = radPerSec;
    }

    public double getTargetRadPerSec() {
        return targetRadPerSec;
    }

    public double getCurrentRadPerSec() {
        return motor1.getSelectedSensorVelocity() * Constants.shooterRadPerPulse * Constants.shooter100msPerS; //TODO Ensure this correct units
    }

    private void runMotors(double output) {
        //TODO: consider if we ever need percentOutput
        if(output < 0.1) {
            motor1.set(ControlMode.PercentOutput, output);
        } else {
            motor1.set(ControlMode.Velocity, output);
        }
    }

    public boolean readyToFire() {
        //TODO: maybe check alignment somewhere else like the drivetrain?
        if(MathUtils.closeEnough(getCurrentRadPerSec(), targetRadPerSec, 15)){//&& inShootingRange() && Robot.m_eye.onTarget();){
            readyCounter++;
        }
        else{
            readyCounter=0;
        }
        return readyFlag;
        
    }

    private double changeInSpeed() {
        return getCurrentRadPerSec() - previousRadPerSec;
    }

    public boolean shotBall() {
        return changeInSpeed() > Constants.shooterChangeInSpeedShot;
    }

    //private void sendToDashboard() {
        // targetRadPerSec = readRadPerSec.getDouble(1.0);
        //currentFlywheelSpeedEntry.setDouble(getCurrentRadPerSec());
        // targetFlywheelSpeedEntry.setDouble(motor1.getMotorOutputVoltage()); UNCOMMENT MAYBE
        //readyToFireEntry.setBoolean(readyToFire());
        // stateEntry.setString(currentState.getName());//
        // ticksEntry.setDouble(getCurrentRadPerSec());
        // targetVEntry.setDouble(calcOutput(targetRadPerSec));
        // limelightDistance.setDouble(m_eye.getDistanceFromTarget());
        // sensorValue.setDouble(motor1.getSelectedSensorVelocity());
        // targetRPM.setDouble(targetRadPerSec*60/(2*Math.PI));
        // outputRPM.setDouble(ticksEntry.getDouble(0)*60/(2*Math.PI));
    //}

    @Override
    public void periodic() {
        previousRadPerSec = getCurrentRadPerSec();
        handleStates();
        runMotors(output);
        //sendToDashboard();
    }
}
