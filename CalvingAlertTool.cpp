#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstdlib>

using namespace std;

const float temperatureThreshold = 35.0;
const float movementThreshold = 2.0;
const int distanceThreshold = 15;

float temperature = 0.0;
float movement = 0.0;
int distance = 0;

mutex tempMutex, movementMutex, distanceMutex;

float simulateTemperature() {
    return (rand() % 100) / 2.0;
}

float simulateMovement() {
    return (rand() % 100) / 50.0;
}

int simulateDistance() {
    return rand() % 30 + 1;
}

void temperatureThread() {
    while (true) {
        float temp = simulateTemperature();
        {
            lock_guard<mutex> lock(tempMutex);
            temperature = temp;
        }
        this_thread::sleep_for(chrono::seconds(2));
    }
}

void accelerometerThread() {
    while (true) {
        float accel = simulateMovement();
        {
            lock_guard<mutex> lock(movementMutex);
            movement = accel;
        }
        this_thread::sleep_for(chrono::seconds(2));
    }
}

void ultrasonicThread() {
    while (true) {
        int dist = simulateDistance();
        {
            lock_guard<mutex> lock(distanceMutex);
            distance = dist;
        }
        this_thread::sleep_for(chrono::seconds(2));
    }
}

void alertThread() {
    int alertCounter = 0;
    while (true) {
        float currentTemp, currentMovement;
        int currentDistance;

        {
            lock_guard<mutex> lock(tempMutex);
            currentTemp = temperature;
        }
        {
            lock_guard<mutex> lock(movementMutex);
            currentMovement = movement;
        }
        {
            lock_guard<mutex> lock(distanceMutex);
            currentDistance = distance;
        }

        if (currentTemp > temperatureThreshold && currentMovement > movementThreshold && currentDistance < distanceThreshold) {
            alertCounter += 5;
        } else {
            alertCounter = 0;
        }

        if (alertCounter >= 300) {
            cout << "ALERT: Cow is likely to calve soon. Temperature: " << currentTemp
                 << "°C, Movement: " << currentMovement << "g, Distance: " << currentDistance << "cm\n";
            alertCounter = 0;
        } else {
            cout << "Monitoring. Temperature: " << currentTemp
                 << "°C, Movement: " << currentMovement << "g, Distance: " << currentDistance << "cm\n";
        }

        this_thread::sleep_for(chrono::seconds(5));
    }
}

int main() {
    srand(time(0));
    thread tempThread(temperatureThread);
    thread accelThread(accelerometerThread);
    thread ultraThread(ultrasonicThread);
    thread alertCheckThread(alertThread);

    tempThread.join();
    accelThread.join();
    ultraThread.join();
    alertCheckThread.join();

    return 0;
}