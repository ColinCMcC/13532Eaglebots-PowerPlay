package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    !----------!-------------
    !          !move to cone, read, 90
    !          !move to pole, drop
    !     v    !back, -90, move to position
    !----------!-------------P------------
    !          !             V v
    !    1     !      x      ^ <R>  3
    !          !      V >    V
    !          !
    !----------!-------------!------------
    !          !             !
    !          !             !
    !          !     V       !
    !----------!-------------!------------
*/

@Autonomous

//sets program name to EaglebotAuto_v5 and includes linearOpMode
public class EaglebotAuto extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig Eagle = new EaglebotConfig(this);

    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        double IsIt123 = 0;
        Eagle.init();// initialize hardware and sensors
       //Eagle.claw.setPosition(0.0);// open claw
        if (!Eagle.home.isPressed())// check for home position
        {
            Eagle.liftHome();// check or go all the way down to home
        }
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous:", "Ready");
        telemetry.update();
        // Waits until start is pressed
        waitForStart();
        if(opModeIsActive()) {
            Eagle.claw.setPosition(1.0);// close claw on cone
            sleep(600);// wait for movement
            LiftUp(400);
            // when in front of signal stop
           // imuDrive(0.8, 400);
            // read color and save
            IsIt123 = Eagle.colorSense();
            sleep(500); // wait to get color
            telemetry.addData("position", IsIt123);
            telemetry.update();
            sleep(5000);
        /*  imuDrive(-0.8, 300);// back up to be able to rotate
            Eagle.move(0,0,.4,false);// stop robot
            while(getHeading() > -90.0)// rotate 90 to kick signal
            { idle(); }
            Eagle.move(0,0,0,false);// stop robot
            imuDrive(-0.8, 300);// move forward
            // get direction to rotate from left right distance sensor

            Eagle.move(0,0,-.4,false);// stop robot
            while(getHeading() < 0.0)// rotate 90 to pole
            { idle(); }
            Eagle.move(0,0,0,false);// stop robot
            // Lift to 25" 4pt pole 12000
            LiftUp(6000);
            imuDrive(.7,150);
            sleep(500);
            Eagle.claw.setPosition(0.0);// release cone
            sleep(500);// wait for claw to open
            Eagle.move(.4,0.0,0.0,false);// backup
            sleep(400);//backup
            Eagle.move(0,0,-.4,false);// stop robot

            while(getHeading() > -90.0)// rotate inside mat
            {
               /* telemetry.addData("heading",getHeading() );
                telemetry.update();
                idle();
            }
            Eagle.move(0,0,0.0,false);// stop robot
        */
            // move to position
            //Eagle.liftHome();// check or go all the way down to home
            // move to number position
        }// if or while autonomous
    }
        // lift cone to 4 point 23.5 inch post
    private void LiftUp(int target)
    {
        // lift(double liftCtrl, double clawCtrl, boolean slow)
        Eagle.liftMotor.setTargetPosition(target);
        Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Eagle.liftMotor.setPower(0.4); // start run to position
        while (opModeIsActive() && Eagle.liftMotor.isBusy()) {
            idle();// let other tasks run
            telemetry.addData("Target", target);
            telemetry.addData("Lift", Eagle.liftMotor.getCurrentPosition());
            telemetry.update();
        }
            Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Eagle.liftMotor.setPower(0.0);
    }


    public double getHeading(){
        Orientation angles = Eagle.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        /* if positive turning right else if negative turning left
         * */
        return heading;// in degrees
    }//end getHeading

    //drive by IMU
    public void imuDrive(double speed,int EncCnt)
    {
        double drift = 0.0;
        int targPos = Eagle.BLMotor.getCurrentPosition() - EncCnt;

        while(opModeIsActive() && (Eagle.BLMotor.getCurrentPosition() > targPos))
        {
            /*telemetry.addData("targPos",targPos );
            telemetry.addData("presPos",Eagle.BLMotor.getCurrentPosition() );
            telemetry.update();*/
            if(getHeading() > .5) drift = .05; else drift = 0.0; // left drift right
            if(getHeading() < -.5) drift = -.05; else drift = 0.0; // right drift left
            Eagle.move(-0.7 + drift, 0.0, 0.0, false);
        }
        Eagle.move(0.0, 0.0, 0.0, false);// stop
    }
}//end EagleAuto class