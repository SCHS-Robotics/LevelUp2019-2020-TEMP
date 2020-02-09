package org.firstinspires.ftc.teamcode.maincode;

import android.media.MediaPlayer;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.SCHSRobotics.HAL9001.util.misc.BeatBox;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;

@TeleOp(name = "Teleop", group = "competition")
public class TeleopProgram extends BaseTeleop {

    private static final String SONG_BUTTON = "soundfxButton";

    public @MainRobot Cygnus robot;
    private CustomizableGamepad gamepad;
    private BeatBox beatBox;
    private Toggle songToggle;
    private int songNum;
    private String currentSong;

    private final static int NUMBER_OF_SONGS = 16;

    @Override
    protected void onInit() {
        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(SONG_BUTTON,new Button(1, Button.BooleanInputs.x));

        songNum = 0;

        songToggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);

        beatBox = new BeatBox();

        beatBox.addSong("Spooky Scary Skeletons (Remix)",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.spookyskeleboys));
        beatBox.addSong("Dual of Fates",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.dualoffates));
        beatBox.addSong("Ra Ra Rasputin",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.rararasputin));
        beatBox.addSong("Gaston",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.gastondos));
        beatBox.addSong("Giorno's Theme",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.ggtheme));
        beatBox.addSong("Inferno - FIREFORCE OP",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.inferno));
        beatBox.addSong("Istanbul Not Constantinople",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.istanbul));
        beatBox.addSong("Still Alive",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.stillalive));
        beatBox.addSong("Imperial March",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.imperialmarch));
        beatBox.addSong("We Are Number One (Remix)",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.wenumberone));
        beatBox.addSong("Still Alive (Swing Version)",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.stillaliveswing));
        beatBox.addSong("Space Mountain Theme",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.spacemountain));
        beatBox.addSong("Gas Gas Gas",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.gasgasgas));
        beatBox.addSong("SPAAACE",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.space));
        beatBox.addSong("Top 10 Numbers",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.toptennumbers));
        beatBox.addSong("Deja Vu",MediaPlayer.create(robot.hardwareMap.appContext, R.raw.dejavudos));

    }

    @Override
    public void onUpdate() {
        songToggle.updateToggle(gamepad.getBooleanInput(SONG_BUTTON));

        if (songToggle.getCurrentState()) {
            String songToPlay = "";
            switch (songNum) {
                case 0:
                    songToPlay = "Spooky Scary Skeletons (Remix)";
                    break;
                case 1:
                    songToPlay = "Dual of Fates";
                    break;
                case 2:
                    songToPlay = "Ra Ra Rasputin";
                    break;
                case 3:
                    songToPlay = "Gaston";
                    break;
                case 4:
                    songToPlay = "Giorno's Theme";
                    break;
                case 5:
                    songToPlay = "Inferno - FIREFORCE OP";
                    break;
                case 6:
                    songToPlay = "Istanbul Not Constantinople";
                    break;
                case 7:
                    songToPlay = "Still Alive";
                    break;
                case 8:
                    songToPlay = "Imperial March";
                    break;
                case 9:
                    songToPlay = "We Are Number One (Remix)";
                    break;
                case 10:
                    songToPlay = "Still Alive (Swing Version)";
                    break;
                case 11:
                    songToPlay = "Space Mountain Theme";
                    break;
                case 12:
                    songToPlay = "Gas Gas Gas";
                    break;
                case 13:
                    songToPlay = "SPAACE";
                    break;
                case 14:
                    songToPlay = "Top 10 Numbers";
                    break;
                case 15:
                    songToPlay = "Deja Vu";
                    break;
            }
            if(currentSong != null) {
                beatBox.stopSong(currentSong);
            }
            currentSong = songToPlay;
            beatBox.playSong(songToPlay);
            songNum++;
            songNum = songNum % NUMBER_OF_SONGS;

            telemetry.addData("song num", songNum);
            telemetry.update();
        }
    }
}
