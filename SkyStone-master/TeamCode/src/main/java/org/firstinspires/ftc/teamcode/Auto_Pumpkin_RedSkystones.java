package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Double.NaN;

/*
 * Created by Sam on 2/27/2020
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto_Pumpkin: RED Skystone", group="Pumpkin: RED")
public class Auto_Pumpkin_RedSkystones extends LinearOpMode {
    Hardware_MecanumUPDATED autopumpkin = new Hardware_MecanumUPDATED();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double REGDRIVE_SPEED = 0.5;
    static final double SLOWDRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
    int stoneCount = 0;

    public Hardware_MecanumUPDATED.Direction direction;
    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        autopumpkin.init(hardwareMap);
        autopumpkin.rotateArmOut();
        autopumpkin.openClaw();

        waitForStart();

        /*
            starts with BACK facing towards foundation side
        */


        //move forwardstoward building site
        movement(.5, .5, .5, .5);
        sleep(1000);// CHECK THIS VALUE FOR GOING BACK FAR/CLOSE ENOUGH
        movement(0, 0, 0, 0);
        sleep(100);

        //move left towards skystones, wait until it sees it
        NSmoveLeftTowardsStones();
        sleep(400);// pause

        //move forwards to find skystone and grab it once found
        while (!autopumpkin.isSkystone()) {
            movement(0.3, 0.3, 0.3, 0.3);
        }
        //sleep(100);// CHECK THIS VALUE FOR MIDDLE OF STONE
        movement(0,0,0,0);

        grabStone();

        //move towards foundation side, drop the stone, reset positions
        SENSmoveTowardsFoundation();
        dropStone();

        //go park next to bridge
        while (!autopumpkin.parkRed()) {
            movement(.5, .5, .5, .5);
        }


        movement(0, 0, 0, 0);
        //end program and ensure stop
        autopumpkin.stopAllMotors();


        /*
            end of program
        */

    }

    public void grabStone() {
        //grab stone
        rotateMotorArmOut();
        autopumpkin.closeClaw();
        sleep(1200);

        // getState() returns TRUE if NOT PRESSED
        while ( autopumpkin.touchSensor.getState()){
            autopumpkin.openClawSlight();
            sleep(500);
            movement(-0.3,0.3,0.3,-0.3);//move left slightly
            sleep(300);
            movement(0.3,0.3,0.3,0.3); //move forwards slightly)
            sleep(150);
            movement(0,0,0,0);

            autopumpkin.FourBarmotor.setPower(.3);
            sleep(100);
            autopumpkin.FourBarmotor.setPower(0);

            autopumpkin.closeClaw();
            sleep(1200);
            stoneCount += 1;
        }

        //lift stone
        rotateMotorArmWSS();

        //if we had to adjust to get the Stone
        if ( stoneCount > 0 ){
            movement(0.3, -0.3, -0.3, 0.3);
            int sleepCount = stoneCount * 300;
            sleep( sleepCount);
            movement(0,0,0,0);
        }
    }

    public void dropStone() {
        //put back stone
        rotateMotorArmOut();

        //drop stone
        autopumpkin.openClaw();
        sleep(500);

        //first reset
        rotateMotorArmIn();
        autopumpkin.closeClaw();
        sleep(750);
        autopumpkin.rotateArmIn();
        sleep(750);
    }

    public void SENSmoveRightTowardsStones() {

        // CHECK THIS FOR DISTANCE UNIT
        while (autopumpkin.stoneDistance.getDistance(DistanceUnit.CM) > 25 || autopumpkin.stoneDistance.getDistance(DistanceUnit.CM) == NaN) {
            movement(SLOWDRIVE_SPEED, -SLOWDRIVE_SPEED, -SLOWDRIVE_SPEED, SLOWDRIVE_SPEED);
            telemetry.addData("this ran",autopumpkin.stoneDistance.getDistance(DistanceUnit.CM));
        }
        movement(0, 0, 0, 0);
    }

    public void NSmoveLeftTowardsStones() {
        movement(-0.5, 0.5, 0.5, -0.5);
        sleep(1400);// CHECK THIS FOR MOVEMENT TOWARDS STONES
        movement(0, 0, 0, 0);
    }

    public void SENSmoveTowardsFoundation() {
        //looks for tape to go over
        while (!autopumpkin.parkRed()) {
            movement(-SLOWDRIVE_SPEED, -SLOWDRIVE_SPEED, -SLOWDRIVE_SPEED, -SLOWDRIVE_SPEED);
        }
        sleep(750);// CHECK THIS VALUE TO MAKE SURE IT MOVES PAST THE BRIDGE TAPE
        movement(0, 0, 0, 0);
    }

    public void NSmoveTowardsFoundation() {
        movement(.5, .5, .5, .5);
        sleep(2000);// CHECK THIS VALUE TO MAKE SURE IT MOVES PAST THE BRIDGE TAPE
        movement(0, 0, 0, 0);
    }

    public void movement(double LF, double LB, double RF, double RB) {
        autopumpkin.LFmotor.setPower(LF);
        autopumpkin.LBmotor.setPower(LB);
        autopumpkin.RFmotor.setPower(RF);
        autopumpkin.RBmotor.setPower(RB);
    }

    public void rotateMotorArmIn() {
        autopumpkin.FourBarmotor.setPower(-.3);
        sleep(1600);
        autopumpkin.FourBarmotor.setPower(0);

    }

    public void rotateMotorArmWSS() {
        autopumpkin.FourBarmotor.setPower(-.5);
        sleep(1600);
        autopumpkin.FourBarmotor.setPower(0);
    }

    public void rotateMotorArmOut() {
        autopumpkin.FourBarmotor.setPower(.3);
        sleep(autopumpkin.getMotorArmSleep());
        autopumpkin.FourBarmotor.setPower(0);
    }

}

