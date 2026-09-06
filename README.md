# SAAR-Fleet: Solar Autonomous Agri-Rover 🚜☀️

## Project Overview
The SAAR-Fleet (Solar Autonomous Agri-Rover) is an autonomous, off-road robotics platform built for precision agriculture. The system operates on dual ESP32-S3 microcontrollers communicating across a dedicated long-range 433MHz LoRa network. 

Instead of traditional uniform field spraying, SAAR-Fleet conducts localized soil diagnostics using an industrial 7-in-1 RS485 probe to analyze Nitrogen, Phosphorus, Potassium (NPK), pH, temperature, and moisture levels in real time. It then deploys dual independent peristaltic lines to deliver targeted micro-doses of water and liquid nutrients directly to plant roots.

---

## Hardware Architecture ⚙️

* **Primary Compute & Telemetry:** Dual ESP32-S3 boards. One manages onboard navigation, sensor polling, and motor actuation; the second serves as an off-field remote/telemetry receiver. An ESP32-CAM provides direct video feedback.
* **Autonomous Navigation:** Path tracking is driven by a uBlox NEO-M8N GPS module with an active ceramic antenna and a QMC5883L 3-axis digital compass. Obstacle avoidance is handled by a 6-node array of VL53L1X Time-of-Flight (ToF) laser LiDAR sensors covering 360 degrees.
* **Drivetrain:** A 4WD all-terrain chassis driven by four independent BTS7960 43A H-Bridge motor drivers to provide high-torque skid-steering across uneven ground.
* **Agricultural Dosing:** An industrial 7-in-1 RS485 sensor interfaced through a MAX485 TTL converter provides continuous soil metrics. Two 12V Kamoer peristaltic pumps run via a 4-channel isolated relay to deliver chemical-resistant, non-clogging liquid delivery.
* **Power Management:** High-discharge 18650 lithium cells regulated by a dedicated BMS, supplemented by an active dual-axis solar tracking array (servo-actuated with LDR tracking) to extend operational cycles.

---

## Build Execution Plan 🛠️
1. Assemble the 4WD chassis and complete individual high-current wiring for the four BTS7960 drivers.
2. Configure I2C address reassignment and multiplexing for the 6-sensor VL53L1X LiDAR ring.
3. Establish bidirectional serial packet transfer over the 433MHz LoRa link between the primary rover controller and the base station ESP32-S3.
4. Integrate the MAX485 converter and write the Modbus-RTU parser to poll the 7-in-1 soil probe registers.
5. Calibrate relay triggers for the peristaltic dosing pumps based on live pH and NPK thresholds.
