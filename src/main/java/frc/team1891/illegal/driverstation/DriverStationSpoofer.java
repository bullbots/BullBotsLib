package frc.team1891.illegal.driverstation;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;

@SuppressWarnings("unused")
public class DriverStationSpoofer {
    static {
        System.out.println("WARNING: DriverStationSpoofer is not FRC legal.  Do not use in a match, and use with caution.");
    }

    private DriverStationSpoofer() {}

    // Configuration constants
    private static final int SOCKET_TIMEOUT_MS = 100;
    private static final int PACKET_SEND_INTERVAL_MS = 20;
    private static final int MAX_SOCKET_CREATION_RETRIES = 10;
    private static final int INITIAL_RETRY_DELAY_MS = 100;
    private static final int MAX_RETRY_DELAY_MS = 5000;
    private static final int CONSECUTIVE_ERRORS_BEFORE_RECONNECT = 5;
    private static final int INIT_DISABLED_PACKET_COUNT = 50;

    private static Thread thread;
    private static boolean isSpoofing = false;
    private static volatile long lastSuccessfulSend = 0;
    private static volatile int consecutiveErrors = 0;

    /**
     * Creates a DatagramSocket with exponential backoff retry logic.
     * @return A configured DatagramSocket, or null if creation failed after all retries
     */
    private static DatagramSocket createSocketWithRetry() {
        int retryDelay = INITIAL_RETRY_DELAY_MS;
        for (int attempt = 0; attempt < MAX_SOCKET_CREATION_RETRIES; attempt++) {
            try {
                DatagramSocket socket = new DatagramSocket();
                socket.setSoTimeout(SOCKET_TIMEOUT_MS);
                System.out.println("DriverStationSpoofer: Socket created successfully" +
                    (attempt > 0 ? " after " + attempt + " retries" : ""));
                return socket;
            } catch (SocketException e) {
                System.err.println("DriverStationSpoofer: Failed to create socket (attempt " +
                    (attempt + 1) + "/" + MAX_SOCKET_CREATION_RETRIES + "): " + e.getMessage());
                if (attempt < MAX_SOCKET_CREATION_RETRIES - 1) {
                    try {
                        Thread.sleep(retryDelay);
                        retryDelay = Math.min(retryDelay * 2, MAX_RETRY_DELAY_MS);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        return null;
                    }
                }
            }
        }
        System.err.println("DriverStationSpoofer: Failed to create socket after all retries");
        return null;
    }

    /**
     * Creates a data packet that mimics what a remote DriverStation would produce.
     */
    private static void generateEnabledDsPacket(byte[] data, short sendCount) {
        data[0] = (byte) (sendCount >> 8);
        data[1] = (byte) sendCount;
        data[2] = 0x01; // general data tag
        data[3] = 0x04; // teleop enabled
        data[4] = 0x10; // normal data request
        data[5] = 0x00; // red 1 station
    }

    /**
     * Enables the robot in teleop from Red Alliance station 1.
     */
    public static void enable() {
        if (RobotBase.isReal()) {
            if (thread != null) {
                thread.interrupt();
            }

            thread = new Thread(() -> {
                DatagramSocket socket = createSocketWithRetry();
                if (socket == null) {
                    System.err.println("DriverStationSpoofer: Failed to create socket, aborting");
                    return;
                }

                InetSocketAddress address = new InetSocketAddress("127.0.0.1", 1110);
                byte[] sendData = new byte[6];
                DatagramPacket packet = new DatagramPacket(sendData, 0, 6, address);
                short sendCount = 0;
                int initCount = 0;
                consecutiveErrors = 0;
                lastSuccessfulSend = System.currentTimeMillis();

                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        Thread.sleep(PACKET_SEND_INTERVAL_MS);
                        generateEnabledDsPacket(sendData, sendCount++);
                        // ~50 disabled packets are required to make the robot actually enable
                        // 1 is definitely not enough.
                        if (initCount < INIT_DISABLED_PACKET_COUNT) {
                            initCount++;
                            // change data code to tell robot to be disabled
                            sendData[3] = 0;
                        }
                        packet.setData(sendData);
                        socket.send(packet);

                        // Track successful send
                        lastSuccessfulSend = System.currentTimeMillis();
                        consecutiveErrors = 0;

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } catch (SocketTimeoutException e) {
                        // Timeout is expected for UDP, this is not necessarily an error
                        consecutiveErrors++;
                        if (consecutiveErrors >= CONSECUTIVE_ERRORS_BEFORE_RECONNECT) {
                            System.err.println("DriverStationSpoofer: Multiple consecutive timeouts, attempting socket reconnect");
                            socket.close();
                            socket = createSocketWithRetry();
                            if (socket == null) {
                                System.err.println("DriverStationSpoofer: Socket reconnection failed, aborting");
                                break;
                            }
                            consecutiveErrors = 0;
                        }
                    } catch (IOException e) {
                        consecutiveErrors++;
                        System.err.println("DriverStationSpoofer: Send error (" + consecutiveErrors +
                            " consecutive): " + e.getMessage());

                        if (consecutiveErrors >= CONSECUTIVE_ERRORS_BEFORE_RECONNECT) {
                            System.err.println("DriverStationSpoofer: Multiple consecutive errors, attempting socket reconnect");
                            socket.close();
                            socket = createSocketWithRetry();
                            if (socket == null) {
                                System.err.println("DriverStationSpoofer: Socket reconnection failed, aborting");
                                break;
                            }
                            consecutiveErrors = 0;
                        }
                    }

                    // Health check logging
                    long timeSinceLastSuccess = System.currentTimeMillis() - lastSuccessfulSend;
                    if (timeSinceLastSuccess > 1000) {
                        System.err.println("DriverStationSpoofer: WARNING - No successful sends in " +
                            timeSinceLastSuccess + "ms");
                    }
                }

                if (socket != null) {
                    socket.close();
                }
                System.out.println("DriverStationSpoofer: Spoofing thread terminated");
            });
            // Because of the test setup in Java, this thread will not be stopped
            // So it must be a daemon thread
            thread.setDaemon(true);
            thread.start();
        } else {
            DriverStationSim.setEnabled(true);
        }
        isSpoofing = true;
    }

    /**
     * Disables the robot.
     */
    public static void disable() {
        if (RobotBase.isReal()) {
            if (thread == null) {
                return;
            }
            thread.interrupt();
            try {
                thread.join(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else {
            DriverStationSim.setEnabled(false);
        }
        isSpoofing = false;
    }

    /**
     * @return if the spoofing thread is currently running
     */
    public static boolean isEnabled() {
        return isSpoofing;
    }

    /**
     * Gets the number of milliseconds since the last successful packet send.
     * @return milliseconds since last successful send, or -1 if never sent or not spoofing
     */
    public static long getMillisSinceLastSuccessfulSend() {
        if (!isSpoofing || lastSuccessfulSend == 0) {
            return -1;
        }
        return System.currentTimeMillis() - lastSuccessfulSend;
    }

    /**
     * Gets the current number of consecutive errors.
     * @return number of consecutive errors
     */
    public static int getConsecutiveErrors() {
        return consecutiveErrors;
    }

    /**
     * Checks if the connection appears healthy.
     * @return true if spoofing is enabled and connection appears healthy
     */
    public static boolean isConnectionHealthy() {
        if (!isSpoofing) {
            return false;
        }
        long timeSinceLastSend = getMillisSinceLastSuccessfulSend();
        return timeSinceLastSend >= 0 && timeSinceLastSend < 500 &&
               consecutiveErrors < CONSECUTIVE_ERRORS_BEFORE_RECONNECT;
    }
}
