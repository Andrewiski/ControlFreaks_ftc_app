package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;

/**
 * Created by adevries on 11/6/2015.
 */
@TeleOp(name="MechBot", group="Manual")  // @Autonomous(...) is the other common choice
//@Disabled
public class CFMechBotManual extends LinearOpMode {

    /* Declare OpMode members. */
    CFPushBotHardware robot;   // Use a Pushbot's hardware

    private static boolean bothControllersEnabled = false;
    private byte v_neopixels_mode = 0;


    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    float stickdeadzone = .1f;
    boolean buttonXReleased = true;
    boolean buttonYReleased = true;
    boolean buttonBReleased = true;
    boolean buttonAReleased = true;
    boolean buttonStartReleased = true;
    boolean buttonDpadleftReleased = true;
    boolean buttonDpadrightReleased = true;
    boolean buttonDpadUpReleased = true;
    boolean buttonDpadDownReleased = true;
    boolean buttonLeftBumperReleased = true;
    boolean buttonRightBumperReleased = true;

    boolean buttonG2DpadleftReleased = true;
    boolean buttonG2DpadrightReleased = true;
    boolean buttonG2DpadUpReleased = true;
    boolean buttonG2DpadDownReleased = true;
    boolean buttonG2XReleased = true;
    boolean buttonG2YReleased = true;
    boolean buttonG2AReleased = true;
    boolean buttonG2BReleased = true;
    boolean button_rightstick_deadzone = true;
    boolean chaseBlueBall = false;
    int chaseBallState = -1;
    int chaseTurnCount = 0;
    int chaseBallSignature = -1;
    boolean highspeedmode = false;
    ElapsedTime myFliperRetractElapsedTime;
    @Override
    public void runOpMode() throws InterruptedException {

        try {
            /* Declare OpMode members. */
            robot           = new CFPushBotHardware();   // Use a Pushbot's hardware
            robot.isMechDrive = true;
            robot.isDriveAndyMark20 = false;
            robot.init(this);
            robot.sensor_pixy_init();

            robot.setupManualDrive();
            //robot.vuforia_Init();
            // Wait for the game to start (driver presses PLAY)
            //robot.led7seg_timer_init(120);
            robot.run_using_encoders();

            waitForStart();
            //robot.debugOff();
            //robot.led7seg_timer_start(120);
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                try {
                    robot.hardware_loop();

                    if(gamepad1.left_bumper){
                        robot.drive_set_strife_power(-1.0f);
                    }else if(gamepad1.right_bumper){
                        robot.drive_set_strife_power(1.0f);
                    }else {
                        robot.drive_set_power(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
                    }


                    if(gamepad2.left_stick_y > .2 ) {
                        //robot.shoulder_step(-.001);
                    }else if(gamepad2.left_stick_y < -.2)
                    {
                        //robot.shoulder_step(.001);
                    }

                    if(gamepad2.right_stick_y > .2 ) {
                        //robot.slider_step(-100, gamepad2.left_bumper);
                        button_rightstick_deadzone = false;
                    }else if(gamepad2.right_stick_y < -.2)
                    {
                        //robot.slider_step(100,false);
                        button_rightstick_deadzone = false;
                    }else{
                        if(button_rightstick_deadzone == false) {
                            //robot.slider_stop(gamepad2.left_bumper);
                            button_rightstick_deadzone = true;
                        }
                    }


                    if(gamepad1.left_bumper){
                        if(buttonLeftBumperReleased == true) {
                            if (robot.debugMode()){
                                //robot.jewel_toggle();
                            }

                            buttonLeftBumperReleased = false;
                        }
                    }else{
                        buttonLeftBumperReleased = true;
                    }
                    if(gamepad1.right_bumper){
                        if(buttonRightBumperReleased == true) {
                            //highspeedmode = !highspeedmode;
                            buttonRightBumperReleased = false;
                        }
                    }else{
                        buttonRightBumperReleased = true;
                    }

                    if(gamepad1.dpad_up){
                        //robot.hand_open();
                        if(buttonDpadUpReleased == true) {
                            buttonDpadUpReleased = false;
                        }
                    }else{
                        buttonDpadUpReleased = true;
                    }
                    if(gamepad1.dpad_down){
                        //robot.hand_close();
                        if(buttonDpadDownReleased == true) {
                            buttonDpadDownReleased = false;
                        }
                    }else{
                        buttonDpadDownReleased = true;
                    }
//                    if(gamepad1.dpad_left){
//                        if(buttonDpadleftReleased == true) {
//
//                            buttonDpadleftReleased = false;
//                        }
//                        robot.slider_step(100);
//                    }else {
//                        buttonDpadleftReleased = true;
//                    }
//                    if(gamepad1.dpad_right){
//                        if(buttonDpadrightReleased == true) {
//
//                            buttonDpadrightReleased = false;
//                        }
//                        robot.slider_step(-100);
//                    }else {
//                        buttonDpadrightReleased = true;
//                    }


                    /*if(gamepad2.dpad_up){

                        if(buttonG2DpadUpReleased == true) {
                            buttonG2DpadUpReleased = false;
                        }
                        robot.lifter_step(100);
                    }else{
                        if(buttonG2DpadUpReleased == false){
                            robot.lifter_stop();
                        }
                        buttonG2DpadUpReleased = true;

                    }
                    if(gamepad2.dpad_down){

                        if(buttonG2DpadDownReleased == true) {

                            buttonG2DpadDownReleased = false;
                        }
                        robot.lifter_step(-100);
                    }else{
                        if(buttonG2DpadDownReleased == false){
                            robot.lifter_stop();
                        }
                        buttonG2DpadDownReleased = true;
                    }

                    if(gamepad2.dpad_left){
                        if(buttonG2DpadleftReleased == true) {
                            robot.wrist_step(.01);
                            buttonG2DpadleftReleased = false;
                        }
                    }else {
                        buttonG2DpadleftReleased = true;
                    }
                    if(gamepad2.dpad_right){
                        if(buttonG2DpadrightReleased == true) {
                            robot.wrist_step(-.01);
                            buttonG2DpadrightReleased = false;
                        }
                    }else {
                        buttonG2DpadrightReleased = true;
                    }*/
                    if(gamepad1.x ){
                        if(buttonXReleased == true) {

                            //lets go find a blue ball
                            if(chaseBlueBall){
                                chaseBlueBall = false;
                                chaseBallState = -1;
                                //robot.blueled_off();
                            }else{
                                chaseBlueBall = true;
                                chaseBallState = 0;
                                chaseBallSignature = 2;
                                //robot.blueled_on();
                            }
                            buttonXReleased = false;
                        }
                    }else{
                        buttonXReleased = true;
                    }
                    if(gamepad1.b){
                        if(buttonBReleased == true) {
                            //robot.redled_toggle();
                            //highspeedmode = true;
                            buttonBReleased = false;
                        }
                    }else{
                        buttonBReleased = true;
                    }
                    if(gamepad1.start ){
                        if(buttonStartReleased == true) {
                            if(robot.debugMode()){
                                robot.debugOff();
                            }else{
                                robot.debugOn();
                            }
                            buttonStartReleased = false;
                        }
                    }else{
                        buttonStartReleased = true;
                    }

                    if(gamepad2.a){
                            if(buttonG2AReleased == true) {
                                //robot.blockgrabbers_toggle();
                                buttonG2AReleased = false;
                            }
                    }else{
                        buttonG2AReleased = true;
                    }
                    if(gamepad2.x){
                        if(buttonG2XReleased == true) {
                            //robot.rackpinion_load();
                            buttonG2XReleased = false;
                        }
                    }else{
                        if(buttonG2XReleased == false) {
                            //if (robot.catapult_load_complete()) {
                                buttonG2XReleased = true;
                            //}
                        }
                    }
                    if(gamepad2.b){
                        if(buttonG2BReleased == true) {
                            //robot.hand_toggle();
                            buttonG2BReleased = false;
                        }
                    }else{
                        if(buttonG2BReleased == false) {
                            buttonG2BReleased = true;
                        }
                    }
                    if(gamepad1.y){
                        if(buttonYReleased == true) {
                            //robot.sensor_color_enable(true);
                            //robot.sensor_color_led(true);

                            //myFliperRetractElapsedTime = null;
                            buttonYReleased = false;
                        }else {
                            int myColor = 0; // robot.sensor_color_GreatestColor();
                            if (myColor == 0) {
                                //robot.redled_on();
                                //robot.blueled_off();
                            } else if (myColor == 2) {

                                //robot.redled_off();
                                //robot.blueled_on();
                            }else {
                                //robot.redled_off();
                                //robot.blueled_off();
                            }
                        }
                    }else{
                        if(buttonYReleased == false){
                            //robot.sensor_color_led(false);
                            //robot.sensor_color_enable(false);
                            //robot.redled_off();
                            //robot.blueled_off();
                        }
                        buttonYReleased = true;

                    }
                    if(chaseBlueBall){
                        switch(chaseBallState){
                            case 0:
                                if(chaseBlueBall){
                                    for(int i = 0; i < 7; i++){
                                        robot.sensor_pixy_maxsignature_enable(i, false);
                                    }
                                    robot.sensor_pixy_maxsignature_enable(chaseBallSignature, true);
                                }
                                robot.sensor_pixy_enable(true);
                                //robot.sensor_range_enable(true);
                                chaseTurnCount = 0;
                                chaseBallState++;
                                break;
                            case 1:
                                if(robot.sensor_range_get_distance() > 4){
                                    if(robot.sensor_pixy_maxSignatureBlocks(chaseBallSignature).BlockCount == 0){
                                       robot.turn_degrees(90,false,false);
                                        chaseTurnCount++;
                                    }else{
                                        //We see a target lets go get it
                                        chaseBallState = 10;
                                    }
                                }
                                chaseBallState++;
                                break;
                            case 2:
                                if(robot.turn_complete() == false){
                                    if(robot.sensor_pixy_maxSignatureBlocks(chaseBallSignature).BlockCount > 0){
                                        robot.drive_inches_stop();
                                        //We see a target lets go get it
                                        chaseBallState = 10;
                                    }
                                }else{
                                    if(chaseTurnCount >= 4){
                                        chaseBallState++;
                                    }else{
                                        chaseBallState = 1;
                                    }
                                }

                                break;
                            case 3:
                                // no ball found and we have turned 360
                                if(robot.sensor_range_get_distance() > 4){
                                    double inches = robot.sensor_range_get_distance();
                                    if (inches >= 24){
                                        inches = 24;
                                    }else{
                                        inches = inches -5;
                                    }
                                    robot.drive_inches((float)inches,false);
                                    chaseBallState++;
                                }else{
                                    chaseTurnCount = 3;
                                    chaseBallState = 1;
                                }

                                break;
                            case 4:
                                if(robot.drive_inches_complete() == false || robot.sensor_range_get_distance() < 5){
                                    if(robot.sensor_pixy_maxSignatureBlocks(chaseBallSignature).BlockCount > 0){
                                        robot.drive_inches_stop();
                                        //We see a target lets go get it
                                        chaseBallState = 10;
                                    }
                                }else{

                                        chaseBallState = 1;

                                }

                                break;
                            case 10:
                                robot.drive_inches_stop();

                        }
                    }

                    //robot.waitForTick(2);
                }catch(Exception ex){
                    robot.set_error_message("Fatal Error "  + ex.toString());
                }
            } // loop

        }catch(Exception ex){
            robot.set_error_message("Fatal Error "  + ex.toString());
        }
    }


}
