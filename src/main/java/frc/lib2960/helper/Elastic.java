package frc.lib2960.helper;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;

public class Elastic {
    /**
     * Starts the elastic layout server
     */
    public static void startLayoutServer() {
        // TODO Test on real robot
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath() + "/elastic");
    }
}
