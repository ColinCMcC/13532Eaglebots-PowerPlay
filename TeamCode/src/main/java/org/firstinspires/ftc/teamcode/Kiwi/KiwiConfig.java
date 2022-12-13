/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 
 */

package org.firstinspires.ftc.teamcode.Kiwi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and front)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class KiwiConfig {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftDrive  = null;
    private DcMotor rightDrive = null;
    private DcMotor frontDrive = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public KiwiConfig (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /*
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightDrive");
        frontDrive = myOpMode.hardwareMap.get(DcMotor.class, "frontDrive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     *
     * robot motions: rho (amplitude of Joy stick x,y and phi (angle of travel).
     * joy stick x and y are converted to polar coordinates
     * Mapped to motor front, left, right speeds and calculates each motor speed
     * Then sends these power levels to the motors.
     *
     */
     // get x y from gamepad  convert to polar values
    public void driveMovement(double y, double x)
    {
        double vl = rho(x,y); // get vector speed
        double vd = phi(x,y); // get vector degree direction
        drivePower(vl,vd);
    }
    // select the nearest angle

    public void drivePower(double vl, double vd) {
        // angles from X line to front motor 90 deg, left is 210, right 330
        final double fref = 1.571; // 90 deg radians
        final double lref = 3.665; // 210 deg radians
        final double rref = 5.760; // 330 deg radians
	// subract the direction phi from reference take sin multiply by direction rohe
        double front = Math.sin(fref - vd) * vl;
        double left  = Math.sin(lref - vd) * vl;
        double right = Math.sin(rref - vd) * vl;
        
        // Scale the values so neither exceed +/- 1.0
        double maxD = Math.max(Math.abs(left), Math.abs(front));
        maxD = Math.max(Math.abs(right), maxD);
        if (maxD > 1.0)
        {
            left /= maxD;
            front /= maxD;
            right /= maxD;
        }
        // Use existing function to drive wheels.
        setDrivePower(left, right,front);
    }
    
    public void driveRotate(boolean D)
    {
        final double rotPwr = .3;
        double setPwr = 0;
        if(D) setPwr = -rotPwr; else setPwr = rotPwr;
        setDrivePower(setPwr,setPwr,setPwr);
    }
    
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel,double frontWheel) {
        // Output the values to the motor drives.
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
        frontDrive.setPower(frontWheel);
    }
    // converts x y to polar vector length
    public double rho(double x, double y)
    {
        return Math.sqrt((y * y)+(x * x));  
    }
    // converts x y  to polar vector deg
    public double phi(double x, double y)
    {
        return -Math.atan2(y, x); 
    }
}
