package org.firstinspires.ftc.teamcode.ControlFreaks;

import android.media.AudioManager;
import android.media.ToneGenerator;

//import com.qualcomm.ftccommon.DbgLog;


/**
 * Created by adevries on 11/16/2015.
 */
public class AudioEffects {
    static final String logId   = "AudioEffects:";       // Tag identifier in FtcRobotController.LogCat
    private String v_deviceName = "Not Set";

    private ToneGenerator v_tone_generator;
    public AudioEffects() throws Exception
    {
        try{
            v_tone_generator = new ToneGenerator(AudioManager.STREAM_RING, ToneGenerator.MAX_VOLUME);

        }catch (Exception p_exeception)
        {
            debugLogException("Error Creating ToneGenerator", p_exeception);
            v_tone_generator = null;
            throw p_exeception;
        }
    }

    Thread v_playing_thread;
    boolean v_is_playing;
    public boolean play_jingle_bells() {
        if (v_is_playing){
            return false;
        }else{
            v_is_playing = true;
            v_playing_thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    // Moves the current Thread into the background
                    android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_BACKGROUND);
                    jingle_bells();
                    //jingle_bells();
                    //jingle_bells();
                    v_is_playing = false;
                }
            });
            v_playing_thread.start();
            return true;
        }
    }
    int v_note_length = 225;
    private void play_dtmf(int number, int length){

        if (v_tone_generator != null){
            if (length == 0){ v_tone_generator.startTone(number, v_note_length /2);

            }else {
                v_tone_generator.startTone(number, v_note_length * length);
            }
            wait((v_note_length * length) + (int) Math.floor(v_note_length * .25D)) ;
        }

    }
    private void wait_notes(int length){
        wait(v_note_length * length - 50);
    }
    private void wait(int length){
        try{
            synchronized(v_tone_generator) {
                v_tone_generator.wait(length);
            }
        }catch(Exception ex){

        }
    }

    private boolean jingle_bells(){

        play_dtmf(3, 1);
        play_dtmf(3, 1);
        play_dtmf(3, 2);
        wait_notes(1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);
        play_dtmf(3, 2);
        wait_notes(1);

        play_dtmf(3, 1);
        play_dtmf(9, 1);
        play_dtmf(1, 1);
        play_dtmf(2, 1);
        play_dtmf(3, 2);
        wait_notes(1);



        play_dtmf(6, 1);
        play_dtmf(6, 1);
        play_dtmf(6, 1);
        wait_notes(1);
        play_dtmf(6, 1);
        play_dtmf(6, 1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);
        wait_notes(1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);

        play_dtmf(2, 1);
        play_dtmf(2, 1);
        play_dtmf(3, 1);
        play_dtmf(2, 1);
        wait_notes(1);

        play_dtmf(9, 2);
        wait_notes(1);
        play_dtmf(3, 1);

        play_dtmf(3, 1);
        play_dtmf(3, 2);
        wait_notes(1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);
        wait_notes(1);
        play_dtmf(3, 1);
        play_dtmf(9, 1);
        play_dtmf(1, 1);
        play_dtmf(2, 1);
        play_dtmf(3, 1);
        wait_notes(1);
        play_dtmf(6, 1);
        play_dtmf(6, 1);
        play_dtmf(6, 1);
        play_dtmf(6, 0);
        wait_notes(1);
        play_dtmf(6, 1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);
        play_dtmf(3, 1);
        play_dtmf(9, 1);
        play_dtmf(9, 1);
        play_dtmf(6, 1);
        play_dtmf(2, 1);
        play_dtmf(1, 1);
        wait_notes(1);
        return true;
    }



    void debugLogException( String msg, Exception ex){

        String debugMessage = logId + msg;
        if (ex != null) {
            String errMsg = ex.getLocalizedMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            }else{
                debugMessage = debugMessage + " error. is null";
            }
        }else{
            debugMessage = debugMessage + " error is null";
        }
        android.util.Log.d("AudioEffects",debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }
}
