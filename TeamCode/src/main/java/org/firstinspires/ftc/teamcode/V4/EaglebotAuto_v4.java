package org.firstinspires.ftc.teamcode.V4;

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
    1 - Home
*/

@Autonomous

//sets program name to EaglebotAuto_v4 and includes linearOpMode
public class EaglebotAuto_v4 extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig_v4 Eagle = new EaglebotConfig_v4(this);

    @Override

    public void runOpMode()
    {
        Eagle.init();// initialize hardware and sensors
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous:", "Ready");
        telemetry.update();


        waitForStart();// Waits until start is pressed
        Eagle.liftHome();
        Eagle.claw.setPosition(0.0);

        // sees what side it is on and runs appropriate code
        if (Eagle.leftDist.getDistance(DistanceUnit.INCH) < Eagle.rightDist.getDistance(DistanceUnit.INCH)){
            Eagle.colorMoveDistLeft();
        }else{
            Eagle.colorMoveDistRight();
        }

    }// end runOpMode function
}//end EagleAuto class