/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 
 */

package org.firstinspires.ftc.teamcode.Kiwi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode Sample illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear opmode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, drawing from this Sample; select TeleOp.
 *  Also add another new file named RobotHardware.java, drawing from the Sample with that name; select Not an OpMode.
 */

@TeleOp(name="TeleOpKiwi", group="Kiwi")

public class TeleOpKiwi extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    KiwiConfig   Kiwi       = new KiwiConfig(this);
    @Override
    public void runOpMode() {
        // double arm        = 0;
        //double handOffset = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        Kiwi.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Send gamepad to KiwiDrive to calculate power to wheels
            if (gamepad1.left_bumper) Kiwi.driveRotate(false); else
            if (gamepad1.right_bumper)Kiwi.driveRotate(true);  else
            Kiwi.driveMovement(gamepad1.left_stick_y, gamepad1.left_stick_x);
            
            // Send telemetry messages to controls and show robot status
            telemetry.addData("- ", "-------");

            telemetry.addData("rho", "%.2f", Kiwi.rho(gamepad1.left_stick_y, gamepad1.left_stick_x));
            telemetry.addData("phi ", "%.2f", Kiwi.phi(gamepad1.left_stick_y, gamepad1.left_stick_x));
            telemetry.update();
            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
