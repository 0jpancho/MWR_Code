/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team101.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot 
{	
	WPI_TalonSRX rightSlave, leftSlave, rightMaster, leftMaster, liftMaster, liftSlave;	
	Spark climberOne, climberTwo;
	
	Joystick driverOne, driverTwo, operator;
	
	DigitalInput limitSwitchTop;
	
	Compressor compressor;
	
	boolean talonsBraked;
	
	boolean toggleArmsBool;
	boolean togglePusher;
	
	static final boolean OPEN = false;
	static final boolean CLOSE = true;
	
	
	AHRS navX;
	
	DoubleSolenoid cubeArms, cubePusher;
	
	@SuppressWarnings("deprecation")
	NetworkTable sdTable = NetworkTable.getTable("SmartDashboard");
	
	final double TURN_ANGLE_SPEED = 0.25;
	
	SendableChooser<String> chosenPosition = new SendableChooser<String>();
	
	final double wheelCircumference = 6 * Math.PI;
	final double ppr = 4096;
	
	final double inchesPerCount = wheelCircumference / ppr;
	final double countsPerInch = 1 / inchesPerCount;
	
	Timer autonTimer;
	
	// Gamepad axis
	//public static final int kGamepadAxisLeftStickX = 1;
	public static final int kGamepadAxisLeftStickY = 1;
	public static final int kGamepadAxisShoulder = 3;
	public static final int kGamepadAxisRightStickX = 4;
	public static final int kGamepadAxisRightStickY = 5;
	public static final int kGamepadAxisDpad = 6;

	// Gamepad buttons
	public static final int kGamepadButtonA = 1; // Bottom Button
	public static final int kGamepadButtonB = 2; // Right Button
	public static final int kGamepadButtonX = 3; // Left Button
	public static final int kGamepadButtonY = 4; // Top Button
	public static final int kGamepadButtonShoulderL = 5;
	public static final int kGamepadButtonShoulderR = 6;
	public static final int kGamepadButtonBack = 7;
	public static final int kGamepadButtonStart = 8;
	public static final int kGamepadButtonLeftStick = 9;
	public static final int kGamepadButtonRightStick = 10;
	public static final int kGamepadButtonMode = -1;
	public static final int kGamepadButtonLogitech = -1;

	
	@Override
	public void robotInit()
	{	
		autonTimer = new Timer();
		
		rightSlave = new WPI_TalonSRX(8);
		rightMaster = new WPI_TalonSRX(1);
		
		rightMaster.setSafetyEnabled(false);
		rightSlave.setSafetyEnabled(false);
		
		leftSlave = new WPI_TalonSRX(2);
		leftMaster = new WPI_TalonSRX(3);
		
		climberOne = new Spark(0);
		climberTwo = new Spark(1);
		
		compressor = new Compressor(0);
		//compressor.start();
		
		leftMaster.setSafetyEnabled(false);
		leftSlave.setSafetyEnabled(false);
		
		rightSlave.set(ControlMode.Follower, 1);
		leftSlave.set(ControlMode.Follower, 3);
		
		liftMaster = new WPI_TalonSRX(4);
		liftMaster.setNeutralMode(NeutralMode.Brake);
		liftMaster.setInverted(false);
		
		liftSlave = new WPI_TalonSRX(5);
		liftSlave.setNeutralMode(NeutralMode.Brake);
		
		liftSlave.setInverted(false);
		liftSlave.set(ControlMode.Follower, 4);
	
		limitSwitchTop = new DigitalInput(0);
		
		cubeArms = new DoubleSolenoid(2, 3);
		cubePusher = new DoubleSolenoid(0, 1);
		
		driverOne = new Joystick(0);
		driverTwo = new Joystick(1);
		operator = new Joystick(2);
		
		leftSlave.setInverted(true);
		leftMaster.setInverted(true);
		
		navX = new AHRS(SerialPort.Port.kUSB);
		
		chosenPosition.addObject("Left", "Left");
		chosenPosition.addObject("Middle", "Middle");
		chosenPosition.addObject("Right", "Right");
		chosenPosition.addObject("Test", "Test");
		chosenPosition.addObject("Drive to Switch", "Drive to Switch");
		chosenPosition.addObject("Drive to Scale", "Drive to Scale");
		
		SmartDashboard.putData("chosenPosition", chosenPosition);
		
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);	
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
		
		leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
		rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
		
		leftMaster.setSelectedSensorPosition(0, 0, 20);
		rightMaster.setSelectedSensorPosition(0, 0, 20);
		
		leftMaster.setSensorPhase(false);
		rightMaster.setSensorPhase(false);
		
		leftMaster.configNominalOutputForward(0, 20);
		leftMaster.configNominalOutputReverse(0, 20);
		
		leftMaster.configPeakOutputForward(1, 20);
		leftMaster.configPeakOutputReverse(-1, 20);
		
		rightMaster.configNominalOutputForward(0, 20);
		rightMaster.configNominalOutputReverse(0, 20);
		
		rightMaster.configPeakOutputForward(1, 20);
		rightMaster.configPeakOutputReverse(-1, 20);
		
		leftSlave.configPeakOutputForward(1, 20);
		leftSlave.configPeakOutputReverse(-1, 20);
		
		rightSlave.configPeakOutputForward(1, 20);
		rightSlave.configPeakOutputReverse(-1, 20);
	}
	
	@SuppressWarnings("deprecation")
	public void disabledPeriodic() {
		
		robotTelemetry();
	}
	
	@Override
	public void autonomousInit() 
	{
		toggleBrakeMode(true);
		navX.zeroYaw();
		
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		resetDriveEncoders();
		
		autonTimer.stop();
		autonTimer.reset();
		
		//Possibility 1: Left Left Left, start left
		if(gameData.equals("LLL") && chosenPosition.getSelected().equals("Left"))
		{	
			liftLift(5, true);
			driveDistance(100, 0.4, 14);
			toggleArms(true);
			togglePusher(true);
			
			resetDriveEncoders();
		}	
		
		//Possibility 2: Left Left Left, start middle
		else if(gameData.equals("LLL") && chosenPosition.getSelected().equals("Middle"))
		{
			driveDistance(80, 0.5, 14);
			
			/*
			liftLift(2, true);
			driveDistance(10, 1);
			turnAngle(30);
			driveDistance(110, 0.5);
			*/
		}
		
		//Possibility 3: Left Left Left, start right
		else if(gameData.equals("LLL") && chosenPosition.getSelected().equals("Right"))
		{
			driveDistance(80, 0.5, 14);
			
			/*
			liftLift(2, true);
			driveDistance(20, 0.5, 14);
			turnAngle(45);
			driveDistance(40, 0.5, 14);
			turnAngle(-45);
			driveDistance(40, 0.5, 14);
			*/
		}
		
		//Possibility 4: Left Right Left, start left
		else if(gameData.equals("LRL") && chosenPosition.getSelected().equals("Left"))
		{
			liftLift(5, true);
			driveDistance(100, 0.5, 14);
			toggleArms(true);
			togglePusher(true);
		}
		
		//Possibility 5: Left Right Left, start middle
		else if(gameData.equals("LRL") && chosenPosition.getSelected().equals("Middle"))
		{
			/*
			liftLift(2, true);
			driveDistance(10, 1);
			turnAngle(30);
			driveDistance(100, 0.5);
			*/
		}
		
		//Possibility 6: Left Right Left, start right
		else if(gameData.equals("LRL") && chosenPosition.getSelected().equals("Right"))
		{
			driveDistance(80, 0.5, 14);
			/*
			liftLift(2, true);
			turnAngle(45);
			driveDistance(80, 0.5);
			turnAngle(-45);
			driveDistance(40, 0.5);
			*/
		}
		
		//Possibility 7: Right Left Right, start left
		else if(gameData.equals("RLR") && chosenPosition.getSelected().equals("Left"))
		{
			driveDistance(80, 0.5, 14);
			/*
			liftLift(2, true);
			turnAngle(-45);
			driveDistance(80, 0.5);
			turnAngle(45);
			driveDistance(40, 0.5);
			*/
		}
		
		//Possibility 8: Right Left Right, start middle
		else if(gameData.equals("RLR") && chosenPosition.getSelected().equals("Middle"))
		{
			driveDistance(80, 0.5, 14);
			/*
			liftLift(2, true);
			turnAngle(-45);
			driveDistance(100, 0.5);
			*/
		}
		
		//Possibility 9: Right Left Right, start right
		else if(gameData.equals("RLR") && chosenPosition.getSelected().equals("Right"))
		{
			liftLift(5, true);
			driveDistance(100, 0.5, 14);
			toggleArms(true);
			togglePusher(true);
		}
		
		//Possibility 10: Right Right Right, start left
		else if(gameData.equals("RRR") && chosenPosition.getSelected().equals("Left"))
		{
			driveDistance(80, 0.5, 14);
		
			/*
			liftLift(2, true);
			turnAngle(-45);
			driveDistance(80, 0.5);
			turnAngle(45);
			driveDistance(40, 0.5);
			*/
		}
		
		//Possibility 11: Right Right Right, start middle
		else if(gameData.equals("RRR") && chosenPosition.getSelected().equals("Middle"))
		{
			driveDistance(80, 0.5, 14);
			/*
			liftLift(2, true);
			turnAngle(-45);
			driveDistance(110, 0.5);
			*/
		}

		//Possibility 12: Right Right Right, start right
		else if(gameData.equals("RRR") && chosenPosition.getSelected().equals("Right"))
		{
			liftLift(5, true);
			driveDistance(100, 0.5, 14);
			toggleArms(true);
			togglePusher(true);
		}
		
		//Test Mode LLL
		else if(gameData.equals("LLL") && chosenPosition.getSelected().equals("Test"))
		{
			liftLift(5, true);
			driveDistance(90, 0.5, 14);
			toggleArms(true);
			togglePusher(true);
			//liftMaster.set(ControlMode.PercentOutput, -0.325);
			
			/*
			driveDistance(60, 0.5);
			toggleArms(OPEN);
			togglePusher(OPEN);
			toggleArms(CLOSE);
			togglePusher(CLOSE);
			*/
			
			//liftMaster.set(ControlMode.PercentOutput, 0);
		}
		
		else if (gameData.equals("LLL") || gameData.equals("RRR") || gameData.equals("LRL") || gameData.equals("RLR") && chosenPosition.getSelected().equals("Drive to Switch"))
		{
			driveDistance(100, 0.5, 14);
		}
		
		else if (gameData.equals("LLL") || gameData.equals("RRR") || gameData.equals("LRL") || gameData.equals("RLR") && chosenPosition.getSelected().equals("Drive to Scale"))
		{
			driveDistance(120, 0.5, 14);
		}
		
		resetDriveEncoders();
		//Hai
	}
	
	@Override
	public void autonomousPeriodic()
	{

	}
	@Override
	public void teleopPeriodic()
	{	
		
		//Drive train control
		leftMaster.set(ControlMode.PercentOutput, driverOne.getY());
		rightMaster.set(ControlMode.PercentOutput, driverTwo.getY());
		
		//Enables brake mode
		if (driverOne.getTrigger() && driverTwo.getTrigger()) 
		{
			toggleBrakeMode(true);
		}
		
		else 
		{
			toggleBrakeMode(false);
		}
		
		//Prevents lift from moving if top limit switch is active
		if(limitSwitchTop.get() == true && operator.getY() < 0)
		{
			liftMaster.set(ControlMode.Disabled, 0);
		}
		else 
		{
			//Lift Control
			liftMaster.set(ControlMode.PercentOutput, operator.getRawAxis(kGamepadAxisLeftStickY) * 0.75);	
		}
		
		//Toggle compressor activation
		if (driverOne.getRawButton(6))
		{
			compressor.start();
		}
		
		else 
		{
			compressor.stop();
		}
		
		//Toggles arm states
		if (operator.getRawAxis(3) == 1 && !toggleArmsBool)
		{
			if(cubeArms.get() == Value.kReverse || cubeArms.get() == Value.kOff)
				cubeArms.set(Value.kForward);
			
			else if(cubeArms.get() == Value.kForward)
				cubeArms.set(Value.kReverse);
				
			toggleArmsBool = true;
		}
	
		else if(operator.getRawAxis(3) < 1)
			toggleArmsBool = false;
		
		//Toggles pusher states
		if (operator.getRawAxis(2) == 1 && !togglePusher)
		{
			
			if(cubePusher.get() == Value.kReverse || cubePusher.get() == Value.kOff)
				cubePusher.set(Value.kForward);
			
			else if(cubePusher.get() == Value.kForward)
				cubePusher.set(Value.kReverse);
			
			togglePusher = true;
		}
		
		else if (operator.getRawAxis(2) < 1)
			togglePusher = false;
		
		
		//Controls climber
		if (operator.getRawButton(2))
		{
			climberOne.set(1);
			climberTwo.set(1);
		}
		
		else if (operator.getRawButton(kGamepadButtonX))
		{
			climberOne.set(-1);
			climberTwo.set(-1);
		}
		else 
		{
			climberOne.set(0);
			climberTwo.set(0);	
		}
		
		robotTelemetry();
	}

	@Override
	public void testPeriodic()
	{
	
	}

	@SuppressWarnings("deprecation")
	public void turnAngle(float degrees)
	{
		//Turn Counter-clockwise
		while (navX.getYaw() < degrees) 
		{
			robotTelemetry();
			
			leftSlave.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
			leftMaster.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
		
			rightSlave.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
			rightMaster.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
		}
		
		leftSlave.set(ControlMode.PercentOutput, 0);
		leftMaster.set(ControlMode.PercentOutput, 0);
		
		rightSlave.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);

		//Turn Clockwise
		while (navX.getYaw() > degrees)
		{
			robotTelemetry();
			
			leftSlave.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
			leftMaster.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
			
			rightSlave.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
			rightMaster.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
		}
		
		leftSlave.set(ControlMode.PercentOutput, 0);
		leftMaster.set(ControlMode.PercentOutput, 0);
		
		rightSlave.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);
	}
	
	public void toggleBrakeMode(boolean toggleBrake)
	{
		if(toggleBrake)
		{
			leftSlave.setNeutralMode(NeutralMode.Brake);
			leftMaster.setNeutralMode(NeutralMode.Brake);
			
			rightSlave.setNeutralMode(NeutralMode.Brake);
			rightMaster.setNeutralMode(NeutralMode.Brake);
		}
		
		else if (!toggleBrake)
		{
			leftSlave.setNeutralMode(NeutralMode.Coast);
			leftMaster.setNeutralMode(NeutralMode.Coast);
			
			rightSlave.setNeutralMode(NeutralMode.Coast);
			rightMaster.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	public void resetDriveEncoders()
	{
		leftMaster.setSelectedSensorPosition(0, 0, 20);
		rightMaster.setSelectedSensorPosition(0, 0, 20);
		
		leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
		rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
	}
	
	public void driveDistance(double inches, double speed, int timeout)
	{
		double targetCounts = countsPerInch * inches;
		
		Timer thisTimer = new Timer();
		
		resetDriveEncoders();
		
		System.out.println(targetCounts);
		thisTimer.reset();
		thisTimer.start();
		while (this.isEnabled() && thisTimer.get() < timeout)
		{	
			robotTelemetry();
			
			if (targetCounts > leftMaster.getSensorCollection().getQuadraturePosition())
			{
				leftMaster.set(ControlMode.PercentOutput, -speed);
			}
			
			if (targetCounts > rightMaster.getSensorCollection().getQuadraturePosition())
			{
				rightMaster.set(ControlMode.PercentOutput, -speed);
			}
			
			if(targetCounts <= leftMaster.getSensorCollection().getQuadraturePosition() 
					|| targetCounts <= rightMaster.getSensorCollection().getQuadraturePosition())
				break;
		}
		
		leftMaster.set(ControlMode.Disabled, 0);
		rightMaster.set(ControlMode.Disabled, 0);
		
		resetDriveEncoders();
	}
	
	public void liftLift(double duration, boolean upward)
	{	
		autonTimer.start();

		double speed = -0.5;
		
		if (!upward)
			speed = 0.5;
		
		while (autonTimer.get() >= 0 && autonTimer.get() < duration && !limitSwitchTop.get())
		{
			liftMaster.set(ControlMode.PercentOutput, speed);

		}
		
		liftMaster.set(ControlMode.Disabled, 0);
		
		autonTimer.stop();
		autonTimer.reset();
	}
	
	public void toggleArms(boolean armToggle)
	{
		if (armToggle)
		{
			cubeArms.set(Value.kForward);
		}
		
		else if (!armToggle) 
		{
			cubeArms.set(Value.kReverse);
		}
	}
	
	public void togglePusher (boolean pusherToggle)
	{
		if (pusherToggle)
		{
			cubePusher.set(Value.kForward);
		}
		
		else if (!pusherToggle)
		{
			cubePusher.set(Value.kReverse);
		}
	}
	
	public void doNothing(double duration)
	{
		autonTimer.start();
		
		while (autonTimer.get() >= 0 && autonTimer.get() <= duration)
		{
			//Literally do absolutely nothing
		}
		
		autonTimer.stop();
		autonTimer.reset();
		
	}
	
	public void robotTelemetry() 
	{	
		sdTable.putDouble("OrIeNtAtIoN", navX.getYaw());
		sdTable.putBoolean("NavX Connected?", navX.isConnected());
		sdTable.putBoolean("NavX Calibrating?", navX.isCalibrating());
		
		sdTable.putBoolean("Talons Braked?", talonsBraked);
	
		SmartDashboard.putNumber("Left Master Enc Counts", leftMaster.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right Master Enc Counts", rightMaster.getSelectedSensorPosition(0));
		
		SmartDashboard.putBoolean("Top Limit Switch", limitSwitchTop.get());
		
		SmartDashboard.putBoolean("Pusher Toggled?", cubePusher.get() == Value.kForward);
		SmartDashboard.putBoolean("Arms Toggled?", cubeArms.get() == Value.kForward);
		
		//SmartDashboard.putNumber("Current Auton Time", autonTimer.get());
		//System.out.println(autonTimer.get());
		
		//SmartDashboard.putNumber("Lifter Speed", operator.getRawAxis(kGamepadAxisLeftStickY) * 0.75);
	}
}
