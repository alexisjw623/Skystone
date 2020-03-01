package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by Sam on 1/07/2020
 ****************************
 *  HORIZONTAL HUB: HUB 2
 *  LC 0         RC 1
 *        HUB 2
 *  FB 2
 *
 *  HUB 2: address 3
 *  Servo Port 0: rotateClaw
 *  Servo Port 4: claw
 ****************************
 *  VERTICAL HUB: HUB 1
 *  LF 0         RF 1
 *        HUB 1
 *  LB 2         RB 3
 *
 *  HUB 1: address 2
 *  Servo Port 0: blockStealer
 *  Servo Port 5: blockPusher
 ****************************
 */

@TeleOp(name="TeleOp: Vuforia Mecanum Test", group="Linear Opmode")
public class TeleOp_VuforiaMecanumTest extends LinearOpMode{
    Hardware_MecanumTest pumpkin1 = new Hardware_MecanumTest();
    double clawPosition, rotatePosition, pusherPosition, servoSpeed, rotateSpeed, stealerPosition;
    double MIN_POSITION = 0; double MAX_POSITION = 1;
    double LOWER_CLAW_LIMIT = .4; double UPPER_CLAW_LIMIT = 1;
    double colorCondition;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AYuXDc//////AAABmW3DOZFrqkEOjiw1BAdoTSiDqSwYTxPEFBWFpGwtWqnqZrdeP+5+WpMZ4pTzDdHaJX4Y6Axdbk5yYlNFsRlqonurS0GFLcpb6n5nVDhunpVB9pdVwDSg8K55o5jeY95JUBKcdeiwX8ScQj6CONjEZML4gkHukeSOYO0yRmI+vaF3j9TrA0gKOuQzdcLDv/KK+/IvkSadtQs2gT0+ib+Y+G/GonW9RXQly5il/QNytsvfZazjAcSt/nYz3NV6ARcz4dNTOwMGnuwCYyhD/WaLxSiBtSDDle/Jh5Uf4W6+ZX3EZjbvkL4pNxEfmj2BQtW0KR2rJ9PVuLnWoWnwImwztgssY1VZ98UJUApRqfpRZvB9";

    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override
    public void runOpMode() {
        pumpkin1.init(hardwareMap);
        waitForStart();

        clawPosition = .4;
        pusherPosition = MIN_POSITION;

        servoSpeed = .25;
        rotateSpeed = .2;

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();

        while (opModeIsActive()) {


            //rotation - right joystick
            double rightX = -gamepad1.right_stick_x;
            //movement - left joystick
            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;


            //driving
            pumpkin1.LFmotor.setPower(leftY + rightX + leftX );
            pumpkin1.RFmotor.setPower(leftY - rightX - leftX);
            pumpkin1.LBmotor.setPower(leftY + rightX - leftX);
            pumpkin1.RBmotor.setPower(leftY - rightX + leftX);


            /* COMPLIANT WHEELS - left trigger/right trigger */
            double wheelOuttake = gamepad1.right_trigger;
            double wheelIntake = -gamepad1.left_trigger;
            //
            if ( gamepad1.left_trigger > 0 ) {
                pumpkin1.LCompliantmotor.setPower(wheelIntake);
                pumpkin1.RCompliantmotor.setPower(wheelIntake);
            }
            else if ( gamepad1.right_trigger > 0){
                pumpkin1.LCompliantmotor.setPower(wheelOuttake);
                pumpkin1.RCompliantmotor.setPower(wheelOuttake);
            }
            else{
                pumpkin1.LCompliantmotor.setPower(0);
                pumpkin1.RCompliantmotor.setPower(0);
            }


            /* FOUR BAR - right/left bumper */
            boolean raiseBar = gamepad1.left_bumper;
            boolean lowerBar = gamepad1.right_bumper;

            if (raiseBar) {
                pumpkin1.FourBarmotor.setPower(.75);
            }
            else if (lowerBar){
                pumpkin1.FourBarmotor.setPower(-.75);
            }
            else{
                pumpkin1.FourBarmotor.setPower(0);
            }

            // if statement removes chance of ArithmeticException (blue = 0), formula adapted from team 5898's color sensor skystone explanation
            //if (pumpkin1.stoneColorS.blue() != 0) colorCondition = pumpkin1.stoneColorS.red() * pumpkin1.stoneColorS.green() / (pumpkin1.stoneColorS.blue() * pumpkin1.stoneColorS.blue());

            //if the color condition is less than two, it's a skystone
            // note: 11/28/2019
            // low lighting conditions means that it detects the orange of the regular stone > the regular skystone,
            // will need to figure out either distance away from blocks/touch sensor/other method to trigger the color
            // sensor regardless of lighting
            //colorPosition = (colorCondition < 2 ) ? MAX_POSITION : MIN_POSITION;

            /* OPEN/CLOSE CLAW - dpad down/dpad up */
            // open the claw servo using the DPAD_DOWN button
            if (gamepad1.dpad_down && clawPosition > LOWER_CLAW_LIMIT) clawPosition = clawPosition - servoSpeed;
            // close the claw servo using the DPAD_UP button
            if (gamepad1.dpad_up && clawPosition < UPPER_CLAW_LIMIT) clawPosition = clawPosition + servoSpeed;

            /* ROTATE CLAW - dpad left/dpad right */
            // rotate the claw system servo out using the DPAD_LEFT button if not already at the most open position
            if (gamepad1.dpad_left && rotatePosition < MAX_POSITION) rotatePosition = rotatePosition + rotateSpeed;
            // rotate the claw system servo out  using the DPAD_RIGHT button if not already at the most closed position
            if (gamepad1.dpad_right && rotatePosition > MIN_POSITION) rotatePosition = rotatePosition - rotateSpeed;


            /* BLOCK STEALER - y and x */
            // put UP the block stealer servo using the X button
            if (gamepad1.x && stealerPosition < .75) stealerPosition= stealerPosition + servoSpeed;
            // put DOWN the block stealer servo using the Y button
            if (gamepad1.y && stealerPosition > MIN_POSITION) stealerPosition = stealerPosition - servoSpeed;

            /* BLOCK PUSHER - a/b */
            // rotate the block pusher servo OUT using the A button if not already at the most open position
            if (gamepad1.a && pusherPosition < MAX_POSITION) pusherPosition = 1;
            // rotate the block pusher servo IN using the B button if not already at the most in position
            if (gamepad1.b && pusherPosition > MIN_POSITION) pusherPosition = 0;


            // set the servo values
            pumpkin1.claw.setPosition(Range.clip(clawPosition, MIN_POSITION, MAX_POSITION));
            pumpkin1.rotateClaw.setPosition(Range.clip(rotatePosition, MIN_POSITION, MAX_POSITION));
            pumpkin1.blockPusher.setPosition(Range.clip(pusherPosition, MIN_POSITION, MAX_POSITION));
            pumpkin1.blockStealer.setPosition(Range.clip(stealerPosition, MIN_POSITION,MAX_POSITION));


            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }

            /*
             * TELEMETRY
             * sends info back to driver station using telemetry function
             */

            //telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", pumpkin1.distanceCS.getDistance(DistanceUnit.CM)));

            telemetry.addData("CONTROLS", "\nintake: LT   outtake: RT\narmup: RB  armdown: LB\nrotatein: dpad_l  rotateout: dp_r\n\n");
            //servo data
            telemetry.addData("clawPosition", String.format("position=%.2f  actual=%.2f", clawPosition, pumpkin1.claw.getPosition()));
            telemetry.addData("rotatePosition", String.format("position=%.2f  actual=%.2f", rotatePosition, pumpkin1.rotateClaw.getPosition()));
            telemetry.addData("pusherPosition", String.format("position=%.2f  actual=%.2f", pusherPosition, pumpkin1.blockPusher.getPosition()));
            telemetry.addData("stealerPosition", String.format("position=%.2f  actual=%.2f", stealerPosition, pumpkin1.blockStealer.getPosition()));

            //color sensor data
            telemetry.addData("PARK COLOR SENSOR", "");
            telemetry.addData("Alpha", pumpkin1.parkColorS.alpha());
            telemetry.addData("Red  ", pumpkin1.parkColorS.red());
            telemetry.addData("Green", pumpkin1.parkColorS.green());
            telemetry.addData("Blue ", pumpkin1.parkColorS.blue());

            /*telemetry.addData("STONE COLOR SENSOR", "");
            telemetry.addData("Alpha", pumpkin1.stoneColorS.alpha());
            telemetry.addData("Red  ", pumpkin1.stoneColorS.red());
            telemetry.addData("Green", pumpkin1.stoneColorS.green());
            telemetry.addData("Blue ", pumpkin1.stoneColorS.blue());
            */

            telemetry.update();


        }
        targetsSkyStone.deactivate();
    }

}
