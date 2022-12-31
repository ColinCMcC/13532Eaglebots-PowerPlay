package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@Autonomous

//sets program name to EaglebotAuto_v5 and includes linearOpMode
public class EaglebotAuto_Right extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig_v5 Eagle = new EaglebotConfig_v5(this);


    public void runOpMode() {
        Eagle.init();// initialize hardware and sensors

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous:", "Ready");
        telemetry.update();

        // Waits until start is pressed
        waitForStart();
        Eagle.liftHome();
        Eagle.claw.setPosition(0.0);//makes sure claw is all the way open to start

        while (opModeIsActive() && Eagle.Distance.getDistance(DistanceUnit.INCH) > 2 && Eagle.backDist.getDistance(DistanceUnit.INCH) < 48) {// Moves up to near cone while staying straight

            double strafePower = (Eagle.rightDist.getDistance(DistanceUnit.INCH) - 27) / 4;// Calculates power to strafe back to correct position
            double turnPower = Eagle.getHeading() / 50;// Calculates power to spin back to straight
            Eagle.move(-0.3, strafePower, turnPower, false);
            sleep(250);
        }
        Eagle.colorMove(2);
    }// end runOpMode function
}//end EagleAuto class