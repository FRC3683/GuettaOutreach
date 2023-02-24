/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand; Come back to this maybe???
import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class OI {

    XboxController driver;
    XboxController operator;
    private static OI instance;
  
    private OI() {
      driver = new XboxController(0);
      operator = new XboxController(1);
    }
  
    public static OI getInstance() {
      if (instance == null) {
        instance = new OI();
      }
      return instance;
    }
  
    private static final double DEADBAND_RADIUS = 0.15;
    private static double deadband(double jsValue) {
      if (Math.abs(jsValue) < DEADBAND_RADIUS){
        return 0;
      }
      jsValue *= 1.0 / (1.0-DEADBAND_RADIUS);
      return jsValue;
    }
  
    public double getXLeftDriver() {
      return deadband(driver.getLeftX());
    }
    public double getXLeftOperator() {
      return deadband(operator.getLeftX());
    }
  
    public double getYLeftDriver(){
      return deadband(driver.getLeftY());
    }
    public double getYLeftOperator(){
      return deadband(operator.getLeftY());
    }
  
    public double getXRightDriver() {
      return deadband(driver.getRightX());
    }
    public double getXRightOperator() {
      return deadband(operator.getRightX());
    }
  
    public double getYRightDriver() {
      return deadband(driver.getRightY());
    }
    public double getYRightOperator() {
      return deadband(operator.getRightY());
    }
  
    public boolean getAButtonDriver() {
      return driver.getAButton();
    }
    public boolean getAButtonOperator() {
      return operator.getAButton();
    }
  
    public boolean getBButtonDriver() {
      return driver.getBButton();
    }
    public boolean getBButtonOperator() {
      return operator.getBButton();
    }
  
    public boolean getXButtonDriver() {
      return driver.getXButton();
    }
    public boolean getXButtonOperator() {
      return operator.getXButton();
    }
  
    public boolean getYButtonDriver() {
      return driver.getYButton();
    }
    public boolean getYButtonOperator() {
      return driver.getYButton();
    }
  
    public boolean getLeftBumperDriver() {
      return driver.getLeftBumper();
    }
    public boolean getLeftBumperOperator() {
      return operator.getLeftBumper();
    }
  
    public boolean getRightBumperDriver() {
      return driver.getRightBumper();
    }
    public boolean getRightBumperOperator() {
      return operator.getRightBumper();
    }
  
    public boolean getBackButtonDriver() {
      return driver.getBackButton();
    }
    public boolean getBackButtonOperator() {
      return operator.getBackButton();
    }
  
    public boolean getStartButtonDriver(){
      return driver.getStartButton();
    }
    public boolean getStartButtonOperator(){
      return operator.getStartButton();
    }
  
    public boolean getLeftStickButtonDriver(){
      return driver.getLeftStickButton();
    }
    public boolean getLeftStickButtonOperator(){
      return operator.getLeftStickButton();
    }
  
    public boolean getRightStickButtonDriver(){
      return driver.getRightStickButton();
    }  
    public boolean getRightStickButtonOperator(){
      return operator.getRightStickButton();
    }
  
    public double getLeftTriggerDriver(){
      return driver.getLeftTriggerAxis();
    }
    public double getLeftTriggerOperator(){
      return operator.getLeftTriggerAxis();
    }
  
    public double getRightTriggerDriver() {
      return driver.getRightTriggerAxis();
    }
    public double getRightTriggerOperator() {
      return operator.getRightTriggerAxis();
    }
  
    public boolean getDPadLeftDriver(){
      return driver.getPOV() == 270;
    }
    public boolean getDPadLeftOperator(){
      return operator.getPOV() == 270;
    }
  
    public boolean getDPadRightDriver() {
      return driver.getPOV() == 90;
    }
    public boolean getDPadRightOperator() {
      return operator.getPOV() == 90;
    }
  
    public boolean getDPadUpDriver() {
      return driver.getPOV() == 0;
    }
    public boolean getDPadUpOperator() {
      return operator.getPOV() == 0;
    }
  
    public boolean getDPadDownDriver() {
      return driver.getPOV() == 180;
    }
    public boolean getDPadDownOperator() {
      return operator.getPOV() == 180;
    }
  
  
    public boolean getAButtonPressedDriver() {
      return (driver.getAButtonPressed());
    }
    public boolean getAButtonPressedOperator() {
      return (operator.getAButtonPressed());
    }
  
  
    public boolean getBButtonPressedDriver() {
      return (driver.getBButtonPressed());
    }
    public boolean getBButtonPressedOperator() {
      return (operator.getBButtonPressed());
    }
  
    public boolean getXButtonPressedDriver() {
      return (driver.getXButtonPressed());
    }
    public boolean getXButtonPressedOperator() {
      return (operator.getXButtonPressed());
    }
  
    public boolean getYButtonPressedDriver() {
      return (driver.getYButtonPressed());
    }
    public boolean getYButtonPressedOperator() {
      return (operator.getYButtonPressed());
    }
  
    public boolean getLeftBumperPressedDriver() {
      return (driver.getLeftBumperPressed());
    }
    public boolean getLeftBumperPressedOperator() {
      return (operator.getLeftBumperPressed());
    }
  
    public boolean getRightBumperPressedDriver() {
      return (driver.getRightBumperPressed());
    }
    public boolean getRightBumperPressedOperator() {
      return (operator.getRightBumperPressed());
    }
  
    public boolean getBackButtonPressedDriver() {
      return (driver.getBackButtonPressed());
    }
    public boolean getBackButtonPressedOperator() {
      return (operator.getBackButtonPressed());
    }
  
    public boolean getStartButtonPressedDriver() {
      return (driver.getStartButtonPressed());
    }
    public boolean getStartButtonPressedOperator() {
      return (operator.getStartButtonPressed());
    }
  
    public boolean getLeftStickButtonPressedDriver() {
      return (driver.getLeftStickButtonPressed());
    }
    public boolean getLeftStickButtonPressedOperator() {
      return (operator.getLeftStickButtonPressed());
    }
  
    public boolean getRightStickButtonPressedDriver() {
      return (driver.getRightStickButtonPressed());
    }
    public boolean getRightStickButtonPressedOperator() {
      return (operator.getRightStickButtonPressed());
    }
  
    public void rumbleDriver(double power) {
      //driver.setRumble(Joystick.RumbleType.kLeftRumble, power);
      //driver.setRumble(Joystick.RumbleType.kRightRumble, power);
    }
  
    public void rumbleOperator(double power) {
      //operator.setRumble(Joystick.RumbleType.kLeftRumble, power);
      //operator.setRumble(Joystick.RumbleType.kRightRumble, power);
    }

    public boolean shootButtonPressed() {
      return getLeftTriggerDriver() > 0.5;
    }

    public boolean intakeButtonPressed() {
      return getRightTriggerDriver() > 0.5;
    }

    
  }
