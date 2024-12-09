#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <wiringPi.h>
#include <wiringPiI2C.h>

// GPIO pins for sensors
const int dhtPin = 7;              // DHT sensor connected to GPIO7 (wiringPi pin numbering)
const int trigPin = 4;             // Ultrasonic sensor Trig pin (GPIO4)
const int echoPin = 5;             // Ultrasonic sensor Echo pin (GPIO5)

// Threshold values
const float temperatureThreshold = 35.0; // Threshold temperature in Celsius
const float movementThreshold = 2.0;     // Threshold for accelerometer (simulated)
const int distanceThreshold = 15;        // Distance threshold in cm

// Shared sensor data
float temperature = 0.0;
float movement = 0.0;
int distance = 0;

// Mutex for shared data
std::mutex tempMutex, movementMutex, distanceMutex;

// MPU6050 I2C address
const int MPU6050_ADDR = 0x68;

// Function to read temperature and humidity from DHT11/DHT22 sensor
bool readDHTSensor(float &temperature) {
    uint8_t data[5] = {0};
    unsigned int lastState = HIGH;
    unsigned int counter = 0;

    // Set pin to output
    pinMode(dhtPin, OUTPUT);
    digitalWrite(dhtPin, LOW);
    delay(18); // Pull the pin low for 18ms
    digitalWrite(dhtPin, HIGH);
    delayMicroseconds(40);
    pinMode(dhtPin, INPUT);

    // Read data from the sensor
    for (int i = 0; i < 85; i++) {
        counter = 0;
        while (digitalRead(dhtPin) == lastState) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255) break;
        }
        lastState = digitalRead(dhtPin);

        if (counter == 255) break;

        // Ignore first 3 transitions
        if ((i >= 4) && (i % 2 == 0)) {
            data[(i - 4) / 16] <<= 1;
            if (counter > 16) data[(i - 4) / 16] |= 1;
        }
    }

    // Verify checksum
    if ((data[0] + data[1] + data[2] + data[3]) != data[4]) return false;

    // Extract temperature
    temperature = data[2] + data[3] / 10.0;

    return true;
}

// Function to read accelerometer sensor (MPU6050)
float readAccelerometerSensor(int fd) {
    int accelX = wiringPiI2CReadReg16(fd, 0x3B); // Read accelerometer X-axis
    int accelY = wiringPiI2CReadReg16(fd, 0x3D); // Read accelerometer Y-axis
    int accelZ = wiringPiI2CReadReg16(fd, 0x3F); // Read accelerometer Z-axis

    // Convert to G-forces (assuming ±2g sensitivity)
    float accel = accelX / 16384.0;
    return accel; // Return X-axis acceleration
}

// Function to read ultrasonic sensor
int readUltrasonicSensor() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Send Trig pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure Echo pulse
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2; // Convert to distance in cm
    return distance;
}

// Function to send an SMS alert using GSM module
void sendAlert(const std::string &message) {
    std::cout << "ALERT: " << message << std::endl;

    // Send SMS command via serial communication
    system("echo -e \"AT+CMGF=1\\r\" > /dev/ttyS0");          // Set GSM module to text mode
    system("echo -e \"AT+CMGS=\\\"+1234567890\\\"\\r\" > /dev/ttyS0"); // Replace with farmer's number
    system(("echo -e \"" + message + "\\x1A\" > /dev/ttyS0").c_str()); // Send the message
}

// Thread to read temperature sensor
void temperatureThread() {
    while (true) {
        float temp = 0.0;
        if (readDHTSensor(temp)) {
            std::lock_guard<std::mutex> lock(tempMutex);
            temperature = temp;
        } else {
            std::cerr << "Failed to read from DHT sensor!" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// Thread to read accelerometer
void accelerometerThread(int fd) {
    while (true) {
        float accel = readAccelerometerSensor(fd);
        {
            std::lock_guard<std::mutex> lock(movementMutex);
            movement = accel;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// Thread to read ultrasonic sensor
void ultrasonicThread() {
    while (true) {
        int dist = readUltrasonicSensor();
        {
            std::lock_guard<std::mutex> lock(distanceMutex);
            distance = dist;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// Thread to check alerts
void alertThread() {
    while (true) {
        float currentTemp, currentMovement;
        int currentDistance;

        // Access shared variables
        {
            std::lock_guard<std::mutex> lock(tempMutex);
            currentTemp = temperature;
        }
        {
            std::lock_guard<std::mutex> lock(movementMutex);
            currentMovement = movement;
        }
        {
            std::lock_guard<std::mutex> lock(distanceMutex);
            currentDistance = distance;
        }

        // Check thresholds and send an alert if needed
        if (currentTemp > temperatureThreshold || currentMovement > movementThreshold || currentDistance < distanceThreshold) {
            sendAlert("Cow is likely to calve soon. Please assist immediately.");
        } else {
            std::cout << "No signs of calving yet." << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

int main() {
    // Initialize wiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "Error initializing wiringPi!" << std::endl;
        return -1;
    }

    // Initialize GPIO pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(dhtPin, INPUT);

    // Initialize MPU6050 (I2C)
    int fd = wiringPiI2CSetup(MPU6050_ADDR);
    if (fd == -1) {
        std::cerr << "Error initializing MPU6050!" << std::endl;
        return -1;
    }
    wiringPiI2CWriteReg8(fd, 0x6B, 0); // Wake up MPU6050

    // Start threads
    std::thread tempThread(temperatureThread);
    std::thread accelThread(accelerometerThread, fd);
    std::thread ultraThread(ultrasonicThread);
    std::thread alertCheckThread(alertThread);

    // Join threads (to keep the program running)
    tempThread.join();
    accelThread.join();
    ultraThread.join();
    alertCheckThread.join();

    return 0;
}
