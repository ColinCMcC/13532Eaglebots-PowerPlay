package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class EaglebotAuto_backup_Left extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig Eagle = new EaglebotConfig(this);

    ElapsedTime runtime = new ElapsedTime();


    public void runOpMode()
    {
        Eagle.init();// Initialize hardware and sensors

        telemetry.addData("Autonomous:", "Ready");// Send telemetry message to signify robot waiting;
        telemetry.update();

        waitForStart();// Waits until start is pressed

        Eagle.liftHome();
        Eagle.claw.setPosition(0.0);
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 5){
            double turnPower = Eagle.getHeading();
            Eagle.move(0.2, -0.2, turnPower / 20, false);
        }
        Eagle.stopDrive();
    }// end runOpMode function
}//end EagleAuto class