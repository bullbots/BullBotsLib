package frc.team1891.common.jni;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class BullBotsLibJNITest {
    @Test
    public void testInitialize() {
        assertEquals(0, BullBotsLibJNI.initialize());
    }
}
