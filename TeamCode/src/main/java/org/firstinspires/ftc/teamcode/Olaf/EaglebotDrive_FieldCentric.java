package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
/*
    Motor Config
    0 - Lmotor white
    1 - RMotor black
    2 - FMotor red
    3 - BMotor blue
    4 - LiftMotor

    Servo Config
    0 - Claw

    Digital Config
    0 - Home

    I2C Config
    0 - 0- IMU
    0 - 1 - leftDist
    1 - 0 - backDist
    2 - 0 - rightDist
    3 - 0 - V3color
*/

@TeleOp

public class EaglebotDrive_FieldCentric extends LinearOpMode {

    //Lets this program call functions inside of DriveConfig
   EaglebotConfig Eagle = new EaglebotConfig(this);

    @Override

    public void runOpMode() {
            Eagle.init(); // initialize the HARDWARE AND SENSORS

            // Wait for the game to start (driver presses PLAY and runs until STOP is pressed)
            waitForStart();

            while (opModeIsActive()) {
                Eagle.checkData();//sends debug info to datapad

                Eagle.fieldMove(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.a);
                Eagle.lift(-gamepad2.left_stick_y, gamepad2.right_trigger, gamepad2.a);

            }//end while opModeIsActive
    }//end void runOpMode
}//Eagle drive  class