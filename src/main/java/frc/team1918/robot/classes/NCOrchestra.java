package frc.team1918.robot.classes;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

public class NCOrchestra {
    private static NCOrchestra instance;
    private String m_music = "";
    private Orchestra m_orchestra = new Orchestra();
    private boolean flagChanged = false;
    private boolean hasAddedInstruments = false;

    // private TalonFX[] instruments = null;

    /**
	 * Returns the instance of the LightingSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return LightingSubsystem instance
	 */
    public static NCOrchestra getInstance() {
		if (instance == null) instance = new NCOrchestra();
		return instance;
	}


    public StatusCode apply(TalonFX... motorsToApply) {
        StatusCode ret = StatusCode.OK;

        if (!hasAddedInstruments) {
            ret = addInstruments(motorsToApply);
            hasAddedInstruments = true;
        }

        if (flagChanged) {
            ret = m_orchestra.loadMusic(m_music);
            flagChanged = false;
        }

        return ret;
    }

    public boolean isPlaying() { return m_orchestra.isPlaying(); }

    public StatusCode play() {
        return m_orchestra.play();
    }

    public StatusCode stop() {
        return m_orchestra.stop();
    }

    public StatusCode pause() {
        return m_orchestra.pause();
    }

    public NCOrchestra withMusic(String music) {
        if (m_music.compareTo(music) != 0) {
            m_music = music;
            flagChanged = true;
        }

        return this;
    }

    private StatusCode addInstruments(TalonFX... motors) {
        StatusCode retErr = StatusCode.OK;

        // Iterate over drive motors
        for(var motor : motors) {
            var err = m_orchestra.addInstrument(motor);

            if (err.isError()) {
                retErr = err;
            }
        }

        // Returns the last error code if an error has occurred
        return retErr;
    }

    // Songs
    // music/Brawl-Theme.chrp
    // music/Megalovania.chrp
    // music/Rickroll.chrp
    // music/Still-Alive.chrp

    // How to Use it
    // dj.google().onTrue(new InstantCommand(() -> {
    //     if(m_orchestra.isPlaying()) {
    //       m_orchestra.stop();
    //     } else {
    //       m_orchestra.withMusic("music/Still-Alive.chrp").play();
    //     }
    //   }).ignoringDisable(true));
}