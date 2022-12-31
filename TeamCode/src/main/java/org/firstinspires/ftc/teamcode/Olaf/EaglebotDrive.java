package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    Motor Config
    0 - LMotor white
    1 - RMotor black
    2 - FMotor red
    3 - BMotor blue
    4 - LiftMotor

    Servo Config
    0 - Claw

    Digital Config
    1 - Home
*/

@TeleOp

public class EaglebotDrive extends LinearOpMode {

    //Lets this program call functions inside of DriveConfig
   EaglebotConfig Eagle = new EaglebotConfig(this);

    //Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    
    @Override

    public void runOpMode() {
            Eagle.init(); // initialize the HARDWARE AND SENSORS

            // Wait for the game to start (driver presses PLAY and runs until STOP is pressed)
            waitForStart();
      /*  while (opModeIsActive()) {// uncomment for diagnostic mode
            Eagle.checkData();//sends debug info to datapad
        }*/

            Eagle.liftHome();

            while (opModeIsActive()) {
                Eagle.checkData();//sends debug info to datapad

                //stops lift from going too high and breaking the string
                if (Eagle.liftMotor.getCurrentPosition() > 15680) {
                    Eagle.liftMotor.setTargetPosition(15600);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Eagle.liftMotor.setPower(0.5);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Eagle.liftMotor.setPower(0.0);
                }
                if (Eagle.liftMotor.getCurrentPosition() < 0) {
                    Eagle.liftMotor.setTargetPosition(10);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Eagle.liftMotor.setPower(0.5);
                    Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Eagle.liftMotor.setPower(0.0);
                }
                //move(drive, strafe, turn, boolean boost)
                Eagle.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.a);
                //lift(liftCtrl, clawCtrl, boolean slow)
                Eagle.lift(-gamepad2.left_stick_y, gamepad2.right_trigger, gamepad2.a);

            }//end while opModeIsActive

    }//end void runOpMode
}//Eagle drive  class