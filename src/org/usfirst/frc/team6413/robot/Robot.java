package org.usfirst.frc.team6413.robot;
import edu.wpi.first.wpilibj.RobotDrive;
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team6413.robot.AdvancedPIDSource.PIDType;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.usfirst.frc.team6413.robot.PixyCmu5.FrameOrder;
import org.usfirst.frc.team6413.robot.PixyCmu5.PixyFrame;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements PIDOutput {
	XboxController stick;
    PIDController turnController;
    double rotateToObjectRate;
	public int counter = 0;
	
	static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;

	public static CANTalon driveBaseLFM;
    public static CANTalon driveBaseLRM;
    public static CANTalon driveBaseRFM;
    public static CANTalon driveBaseRRM;
	
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	PixyCmu5 pixycam;
	
	RobotDrive drive;
	
	public Robot() {
		/*IterativeRobot = new RobotDrive(frontLeftChannel, rearLeftChannel,
        		frontRightChannel, rearRightChannel);
        IterativeRobot.setExpiration(0.1);
        stick = new XboxController(0);*/
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		driveBaseLFM = new CANTalon(1);
        LiveWindow.addActuator("DriveBase", "LFM", (CANTalon) driveBaseLFM);
        
        driveBaseLRM = new CANTalon(3);
        LiveWindow.addActuator("DriveBase", "LRM", (CANTalon) driveBaseLRM);
        
        driveBaseRFM = new CANTalon(0);
        LiveWindow.addActuator("DriveBase", "RFM", (CANTalon) driveBaseRFM);
        
        driveBaseRRM = new CANTalon(2);
        LiveWindow.addActuator("DriveBase", "RRM", (CANTalon) driveBaseRRM);
        
        
        drive = new RobotDrive(driveBaseLFM, driveBaseLRM, driveBaseRFM, driveBaseRRM);
               
        pixycam = new PixyCmu5(new I2C(I2C.Port.kOnboard, 0x54), 0.02, null);
        
        pixycam.setPIDType(PIDType.ANGLE);
        pixycam.setSortBy(FrameOrder.AREA);
        pixycam.setSortAscending(false);
        pixycam.setMinObjects(1);
        
        stick = new XboxController(0);
        
        turnController = new PIDController(kP, kI, kD, kF, pixycam, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//byte[] buffer = new byte[16];
		//boolean foo = wire.read(84, 16, buffer);
		double currentRotationRate;
		
		List<PixyFrame> frames = pixycam.getCurrentframes();
		
		for(PixyFrame frame : frames) {
			double distance = PixyCmu5.degreesXFromCenter(frame);
			System.out.println("degrees from center: " + distance);
			System.out.println("x: " + frame.xCenter);
			System.out.println("sig: " + frame.signature);
		}
		
		turnController.setSetpoint(0.0f);
		turnController.enable();
		
		currentRotationRate = rotateToObjectRate;
		
		drive.arcadeDrive(0, currentRotationRate);
    } 


	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	public static int convertBytesToInt(int msb, int lsb)
    {
        if (msb < 0)
            msb += 256;
        int value = msb * 256;

        if (lsb < 0)
        {
            // lsb should be unsigned
            value += 256;
        }
        value += lsb;
        return value;
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		rotateToObjectRate = output;
		System.out.println("pid output: " + rotateToObjectRate);
		
	}
}

