package org.usfirst.frc.team1523.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together. The program also delays a
 * short time in the loop to allow other threads to run. This is generally a
 * good idea, especially since the joystick values are only transmitted from the
 * Driver Station once every 20ms.
 */
public class Robot extends SampleRobot {

//	private SpeedController motor = new Talon(0); // initialize the motor as a
													// Talon on channel 0
	private Joystick stick = new Joystick(0); // initialize the joystick on port
												// 0

	private final double kUpdatePeriod = 0.005; // update every 0.005 seconds/5
												// milliseconds (200Hz)

	private static final int SET_TARGET_BUTTON = 1; 
	private static final double MAX_ROTATION = 0.1;
	private static final double ROTATION_THR = 0.25;
	
	private double moveValue;
	private double rotateValue;
	private double lastTargetPosition;
	private double kp = 0.18;  //0.18
	private double kd = 4.0; //4.0
	private double prevError = 0.0;
	
	CANTalon motorL = new CANTalon(12);
	CANTalon motorR = new CANTalon(13);
	
	RobotDrive myRobot = new RobotDrive(motorL, motorR);
	NetworkTable ntable;

	public Robot() {
		motorL.setInverted(true);
		motorR.setInverted(true);
		lastTargetPosition = 0.3;
		
		new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            camera.setExposureManual(-20);
            
            MjpegServer mjpegserver1 = new MjpegServer("serve_USB Camera 0", 1181);
            
//            CvSink cvSink = CameraServer.getInstance().getVideo();
//            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
//            
//            Mat source = new Mat();
//            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
//                cvSink.grabFrame(source);
//                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
//                outputStream.putFrame(output);
            }
        }).start();
		
		ntable = NetworkTable.getTable("GRIP/myContoursReport");
		
	}

	
	@Override
	public void autonomous()
	{
		while (isAutonomous() && isEnabled()){
			double[] defaultValue = new double[0];
			double[] areas = ntable.getNumberArray("area", defaultValue);
			double[] centersX = ntable.getNumberArray("centerX", defaultValue);
			double[] width = ntable.getNumberArray("width", defaultValue);
			
			if (centersX.length > 0){
				
				
				
				double centeravg = 0.0;
				for (double center : centersX){
					centeravg += center;
				}
				centeravg /= centersX.length;
				SmartDashboard.putNumber("TargetWidth", centeravg);
				
				
				
				double TargetCenterX = SmartDashboard.getNumber("TargetCenterX", 0.0);
				double TargetWidth = SmartDashboard.getNumber("TargetWidth", 0.0);
				
				double error = centeravg - TargetCenterX;
				error /= 320;
				lastTargetPosition = error;
				double diffError = error - prevError;
				prevError = error;
				double rotation = kp * error + kd * diffError;
				
				if (rotation > MAX_ROTATION) rotation = MAX_ROTATION;
				if (rotation < -MAX_ROTATION) rotation = -MAX_ROTATION;
				
				if (rotation > 0) rotation += ROTATION_THR;
				if (rotation < 0) rotation -= ROTATION_THR;
				
				SmartDashboard.putNumber("error", error);
				SmartDashboard.putNumber("diffError", diffError);
				SmartDashboard.putNumber("rotateValue", rotation);
				
				myRobot.arcadeDrive(0, rotation,true);
				
//				kp = (double) SmartDashboard.getNumber("kp", 0.2);
//				kd = (double) SmartDashboard.getNumber("kd", 3.0);
				
			}else {
				if (lastTargetPosition > 0.35) lastTargetPosition = 0.35;
				if (lastTargetPosition < -0.35) lastTargetPosition = -0.35;
				myRobot.arcadeDrive(0, lastTargetPosition,true);
			}

			Timer.delay(kUpdatePeriod); // wait 5ms to the next update
		}
	}
	/**
	 * Runs the motor from a joystick.
	 */
	
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			// Set the motor's output.
			// This takes a number from -1 (100% speed in reverse) to +1 (100%
			// speed going forward)
//			motorL.set(stick.getY());
			
			moveValue = stick.getY() * 0.5;
			rotateValue = stick.getX() * 0.5;
			
			myRobot.arcadeDrive(moveValue, rotateValue,true);
			Timer.delay(kUpdatePeriod); // wait 5ms to the next update
		}
	}
}
