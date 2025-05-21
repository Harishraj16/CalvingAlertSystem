import java.util.Random;
import java.util.concurrent.locks.ReentrantLock;

public class CowMonitoringSystem {
    private static final float temperatureThreshold = 35.0f;
    private static final float movementThreshold = 2.0f;
    private static final int distanceThreshold = 15;

    private static float temperature = 0.0f;
    private static float movement = 0.0f;
    private static int distance = 0;

    private static final ReentrantLock tempLock = new ReentrantLock();
    private static final ReentrantLock movementLock = new ReentrantLock();
    private static final ReentrantLock distanceLock = new ReentrantLock();

    private static final Random rand = new Random();

    public static void main(String[] args) {
        Thread tempThread = new Thread(CowMonitoringSystem::temperatureThread);
        Thread accelThread = new Thread(CowMonitoringSystem::accelerometerThread);
        Thread ultraThread = new Thread(CowMonitoringSystem::ultrasonicThread);
        Thread alertThread = new Thread(CowMonitoringSystem::alertThread);

        tempThread.start();
        accelThread.start();
        ultraThread.start();
        alertThread.start();
    }

    private static float simulateTemperature() {
        return rand.nextInt(100) / 2.0f;
    }

    private static float simulateMovement() {
        return rand.nextInt(100) / 50.0f;
    }

    private static int simulateDistance() {
        return rand.nextInt(30) + 1;
    }

    private static void temperatureThread() {
        while (true) {
            float temp = simulateTemperature();
            tempLock.lock();
            try {
                temperature = temp;
            } finally {
                tempLock.unlock();
            }
            sleep(2000);
        }
    }

    private static void accelerometerThread() {
        while (true) {
            float accel = simulateMovement();
            movementLock.lock();
            try {
                movement = accel;
            } finally {
                movementLock.unlock();
            }
            sleep(2000);
        }
    }

    private static void ultrasonicThread() {
        while (true) {
            int dist = simulateDistance();
            distanceLock.lock();
            try {
                distance = dist;
            } finally {
                distanceLock.unlock();
            }
            sleep(2000);
        }
    }

    private static void alertThread() {
        int alertCounter = 0;
        while (true) {
            float currentTemp;
            float currentMovement;
            int currentDistance;

            tempLock.lock();
            try {
                currentTemp = temperature;
            } finally {
                tempLock.unlock();
            }

            movementLock.lock();
            try {
                currentMovement = movement;
            } finally {
                movementLock.unlock();
            }

            distanceLock.lock();
            try {
                currentDistance = distance;
            } finally {
                distanceLock.unlock();
            }

            if (currentTemp > temperatureThreshold && currentMovement > movementThreshold && currentDistance < distanceThreshold) {
                alertCounter += 5;
            } else {
                alertCounter = 0;
            }

            if (alertCounter >= 300) {
                System.out.println("ALERT: Cow is likely to calve soon. Temperature: " + currentTemp +
                        "°C, Movement: " + currentMovement + "g, Distance: " + currentDistance + "cm");
                alertCounter = 0;
            } else {
                System.out.println("Monitoring. Temperature: " + currentTemp +
                        "°C, Movement: " + currentMovement + "g, Distance: " + currentDistance + "cm");
            }

            sleep(5000);
        }
    }

    private static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ignored) {}
    }
}
