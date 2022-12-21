package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

public class EaglebotDrive_v5 extends LinearOpMode {

    //Lets this program call functions inside of DriveConfig
   EaglebotConfig_v5 Eagle = new EaglebotConfig_v5(this);

    @Override

    public void runOpMode() {
            Eagle.init(); // initialize the HARDWARE AND SENSORS

            // Wait for the game to start (driver presses PLAY and runs until STOP is pressed)
            waitForStart();

            Eagle.liftHome();

            while (opModeIsActive()) {
                Eagle.checkData();//sends debug info to datapad

                //stops lift from going too high and breaking the string
                if (Eagle.liftMotor.getCurrentPosition() > 5200) {
                    Eagle.liftMotor.setTargetPosition(5180);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Eagle.liftMotor.setPower(0.5);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Eagle.liftMotor.setPower(0);
                }
                if (Eagle.liftMotor.getCurrentPosition() < 0) {
                    Eagle.liftMotor.setTargetPosition(10);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Eagle.liftMotor.setPower(0.5);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Eagle.liftMotor.setPower(0);
                }

                Eagle.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.a);
                Eagle.lift(-gamepad2.left_stick_y, gamepad2.left_trigger, gamepad2.a);

            }//end while opModeIsActive
    }//end void runOpMode
}//Eagle drive  class