/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.config;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.utils.BNO055;
/**
 * Add your docs here.
 */
public class Config {
    private static Config instance;

    private static CANSparkMax driveMotorLeft1;
    private static CANSparkMax driveMotorLeft2;
    private static CANSparkMax driveMotorRight1;
    private static CANSparkMax driveMotorRight2;
    
    private static Encoder driveLeftEnc;
    private static Encoder driveRightEnc;
    private static BNO055 driveGyro;
  
    private TalonSRX shooterMotor1;
    private VictorSPX shooterMotor2;
    
    private Solenoid shooterHoodPiston;
    
    private VictorSPX intakeRollerMotor;
    private Solenoid intakePiston;
    private VictorSPX intakeConveyorMotor;

    private Config() {
        driveMotorLeft1 = new CANSparkMax(12, MotorType.kBrushless);
        driveMotorLeft2 = new CANSparkMax(13, MotorType.kBrushless);
        driveMotorRight1 = new CANSparkMax(2, MotorType.kBrushless);
        driveMotorRight2 = new CANSparkMax(3, MotorType.kBrushless);

        driveMotorLeft1.clearFaults();
        driveMotorLeft2.clearFaults();
        driveMotorRight1.clearFaults();
        driveMotorRight2.clearFaults();

        driveMotorLeft1.setSmartCurrentLimit(50);
        driveMotorLeft2.setSmartCurrentLimit(50);
        driveMotorRight1.setSmartCurrentLimit(50);
        driveMotorRight2.setSmartCurrentLimit(50);
      
    //     driveLeftEnc = new Encoder(2, 3);
    //     driveLeftEnc.setDistancePerPulse(1.0 / Constants.driveTicksPerInchLeft);
    //     driveLeftEnc.setReverseDirection(false);
    //     driveRightEnc = new Encoder(0, 1);
    //     driveRightEnc.setDistancePerPulse(1.0 / Constants.driveTicksPerInchRight);
    //     driveRightEnc.setReverseDirection(true);
      
        intakeRollerMotor = new VictorSPX(9);
        intakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        intakeConveyorMotor = new VictorSPX(10);
        intakeConveyorMotor.setNeutralMode(NeutralMode.Brake);
        
        shooterMotor1 = new TalonSRX(1);
        shooterMotor1.configVoltageCompSaturation(Constants.shooterMaxVoltage);
        shooterMotor1.enableVoltageCompensation(true);

        shooterMotor2 = new VictorSPX(14);
        //TODO: Figure out if this is needed if the victor is following the talon
        shooterMotor2.configVoltageCompSaturation(Constants.shooterMaxVoltage);
        shooterMotor2.enableVoltageCompensation(true);

        shooterMotor1.setNeutralMode(NeutralMode.Coast);
        shooterMotor2.setNeutralMode(NeutralMode.Coast);

        shooterMotor2.follow(shooterMotor1);
        shooterMotor1.setInverted(true);
        shooterMotor2.setInverted(InvertType.OpposeMaster);

        shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        shooterMotor1.setSensorPhase(true); //Invert this if sensor is out of phase with motor

        shooterMotor1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);
        shooterMotor1.configVelocityMeasurementWindow(64);

        shooterHoodPiston = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    }

    public static Config getInstance() {
        if (instance == null) {
            instance = new Config();
        }
        return instance;
    }

    public TalonSRX getShooterMotor1() {
        return shooterMotor1;
    }

    public VictorSPX getShooterMotor2() {
        return shooterMotor2;
    }

    public Solenoid getShooterHoodPiston() {
        return shooterHoodPiston;
    }
    public VictorSPX getRoller() {
       return intakeRollerMotor;
    }

    public VictorSPX getConveyor() {
        return intakeConveyorMotor;
     }

    public Solenoid getIntakePiston() {
        return intakePiston;
    }

    public static CANSparkMax getDriveMotorLeft1() {
        return driveMotorLeft1;
    }

    public static CANSparkMax getDriveMotorLeft2() {
        return driveMotorLeft2;
    }

    public static CANSparkMax getDriveMotorRight1() {
        return driveMotorRight1;
    }

    public static CANSparkMax getDriveMotorRight2() {
        return driveMotorRight2;
    }

    public static Encoder getDriveLeftEnc() {
        return driveLeftEnc;
    }

    public static Encoder getDriveRightEnc() {
        return driveRightEnc;
    }

    public static BNO055 getDriveGyro() {
        return driveGyro;
    }
}
