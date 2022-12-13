/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 
 */

package org.firstinspires.ftc.teamcode.Dwan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class DwanConfig {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftDrive  = null;
    private DcMotor rightDrive = null;
    private DcMotor frontDrive = null;
    private DcMotor rearDrive  = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * GEAR_RATIO) /
            (WHEEL_RADIUS * 2 * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public DwanConfig(LinearOpMode opmode) {
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
        rearDrive = myOpMode.hardwareMap.get(DcMotor.class, "rearDrive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontDrive.setDirection(DcMotor.Direction.FORWARD);
        rearDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
         leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }//init

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double FwdInches, double StrfInches,
                             double timeoutS)
    {
        int newFwdTarget;
        int newStrfTarget;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newStrfTarget = leftDrive.getCurrentPosition() + (int) (FwdInches * COUNTS_PER_INCH);
            newFwdTarget = rightDrive.getCurrentPosition() + (int) (StrfInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newStrfTarget);
            rightDrive.setTargetPosition(newFwdTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", " %7d :%7d", newStrfTarget, newFwdTarget);
                myOpMode.telemetry.addData("Currently at", " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250);   // optional pause after each move.
        }
    } // drive by encoder

    /**
     *
     * robot motions:
     * joy stick x and y represent power request to motor
     * Mapped to motor front, left, right speeds and calculates each motor speed
     * Then sends these power levels to the motors.
     *
     */
    public void drivePower(double x, double y) {
        // Scale the values so neither exceed +/- 1.0
        double maxD = Math.max(Math.abs(x), Math.abs(y));
        if (maxD > 1.0)
        {
            x /= maxD;
            y /= maxD;
        }
        // Use existing function to drive wheels.
        setDrivePower(x, x, y, y);
    } // drivePower
    
    public void driveRotate(boolean D)
    {
        final double rotPwr = .3;
        double setPwr = 0;
        if(D) setPwr = -rotPwr; else setPwr = rotPwr;
        setDrivePower(setPwr,setPwr,setPwr, setPwr);
    }// driverotate
    
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftWheel, double rightWheel,double frontWheel, double rearWheel) {
        // Output the values to the motor drives.
        leftDrive.setPower(leftWheel);
        rightDrive.setPower(rightWheel);
        frontDrive.setPower(frontWheel);
        frontDrive.setPower(rearWheel);
    }//setDrivePower
}// end class Dwan Config
