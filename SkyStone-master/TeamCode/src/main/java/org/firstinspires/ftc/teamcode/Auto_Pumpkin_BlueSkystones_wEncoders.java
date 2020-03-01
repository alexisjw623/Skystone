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
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto_Pumpkin: BLUE Skystone ENCODERS", group="Tests")
public class Auto_Pumpkin_BlueSkystones_wEncoders extends LinearOpMode {
    Hardware_MecanumUPDATED autopumpkin = new Hardware_MecanumUPDATED();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double REGDRIVE_SPEED = 0.5;
    static final double SLOWDRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;

    public Hardware_MecanumUPDATED.Direction direction;
    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        autopumpkin.init(hardwareMap);
        autopumpkin.rotateArmOut();
        autopumpkin.openClaw();

        waitForStart();

        /*
            starts with FRONT facing towards foundation side
        */


        //move backwards toward building site
        encoderDrive(Hardware_MecanumUPDATED.Direction.STRAFE_LEFT, 0.3, 12,3.5);
        sleep(1000);// CHECK THIS VALUE FOR GOING BACK FAR/CLOSE ENOUGH
        movement(0, 0, 0, 0);
        sleep(100);

        //move right towards skystones, wait until it sees it
        NSmoveRightTowardsStones();
        sleep(500);// pause

        //move backwards to find skystone and grab it once found
        while (!autopumpkin.isSkystone()) {
            movement(-0.2, -0.2, -0.2, -0.2);
        }
        sleep(100);// CHECK THIS VALUE FOR MIDDLE OF STONE
        movement(0,0,0,0);
        grabStone();

        //move towards foundation side, drop the stone, reset positions
        SENSmoveTowardsFoundation();
        dropStone();

        //go park next to bridge
        while (!autopumpkin.parkBlue()) {
            movement(-.5, -.5, -.5, -.5);
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
        sleep(1000);

        //lift stone
        rotateMotorArmWSS();
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

    public void NSmoveRightTowardsStones() {
        encoderDrive(Hardware_MecanumUPDATED.Direction.STRAFE_RIGHT, .5, 10, 2);
        //movement(0.5, -0.5, -0.5, 0.5);
        //sleep(1250);// CHECK THIS FOR MOVEMENT TOWARDS STONES
        movement(0, 0, 0, 0);
    }

    public void SENSmoveTowardsFoundation() {
        //looks for tape to go over
        while (!autopumpkin.parkBlue()) {
            movement(SLOWDRIVE_SPEED, SLOWDRIVE_SPEED, SLOWDRIVE_SPEED, SLOWDRIVE_SPEED);
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

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(Hardware_MecanumUPDATED.Direction direction, double speed,
                             double Inches, double timeoutS) {
        int LFtarget = 0;
        int RFtarget = 0;
        int LBtarget = 0;
        int RBtarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive() && !isStopRequested()) {

            autopumpkin.LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autopumpkin.RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autopumpkin.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autopumpkin.RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Reset the encoder
            autopumpkin.LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            autopumpkin.RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            autopumpkin.LBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            autopumpkin.RBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Ensure that the op mode is still active
            if (opModeIsActive() && !isStopRequested()) {

                // Determine new target position, and pass to motor controller
                if (direction == Hardware_MecanumUPDATED.Direction.FORWARD) {
                    //Go forward
                    LFtarget = autopumpkin.LFmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                    RFtarget = autopumpkin.RFmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                    LBtarget = autopumpkin.LBmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                    RBtarget = autopumpkin.RBmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);


                } else if (direction == Hardware_MecanumUPDATED.Direction.BACKWARD) {
                    //Go backward
                    LFtarget = autopumpkin.LFmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                    RFtarget = autopumpkin.RFmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                    LBtarget = autopumpkin.LBmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                    RBtarget = autopumpkin.RBmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                } else if (direction == Hardware_MecanumUPDATED.Direction.STRAFE_RIGHT) {
                    //Strafe Right
                    LFtarget = autopumpkin.LFmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                    RFtarget = autopumpkin.RFmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                    LBtarget = autopumpkin.LBmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                    RBtarget = autopumpkin.RBmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

                } else if (direction == Hardware_MecanumUPDATED.Direction.STRAFE_LEFT) {
                    //Strafe Left
                    LFtarget = autopumpkin.LFmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                    RFtarget = autopumpkin.RFmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                    LBtarget = autopumpkin.LBmotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                    RBtarget = autopumpkin.RBmotor.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);

                } else {
                    LFtarget = autopumpkin.LFmotor.getCurrentPosition() + (int) (0 * Inches * COUNTS_PER_INCH);
                    RFtarget = autopumpkin.RFmotor.getCurrentPosition() + (int) (0 * Inches * COUNTS_PER_INCH);
                    LBtarget = autopumpkin.LBmotor.getCurrentPosition() + (int) (0 * Inches * COUNTS_PER_INCH);
                    RBtarget = autopumpkin.RBmotor.getCurrentPosition() + (int) (0 * Inches * COUNTS_PER_INCH);
                }

                autopumpkin.LFmotor.setTargetPosition(LFtarget);
                autopumpkin.RFmotor.setTargetPosition(RFtarget);
                autopumpkin.LBmotor.setTargetPosition(LBtarget);
                autopumpkin.RBmotor.setTargetPosition(RBtarget);

                // Turn On RUN_TO_POSITION
                autopumpkin.LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                autopumpkin.RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                autopumpkin.LBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                autopumpkin.RBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                autopumpkin.LFmotor.setPower(Math.abs(speed));
                autopumpkin.RFmotor.setPower(Math.abs(speed));
                autopumpkin.LBmotor.setPower(Math.abs(speed));
                autopumpkin.RBmotor.setPower(Math.abs(speed));


            /*

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }*/

                // Stop all motion;
                autopumpkin.stopAllMotors();

                // Turn off RUN_TO_POSITION
                autopumpkin.LFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                autopumpkin.RFmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                autopumpkin.LBmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                autopumpkin.RBmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                sleep(250);   // pause after each move
            }
        }
    }
}

