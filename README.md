## Romea Core Localisation Library

This library provides robust implementations of three key localization algorithms:

- **Robot-to-World Localization:**
  Localizes a robot within the global environment using sensor data from odometry, IMU, GPS, LIDAR (coming soon), RADAR, and RTLS (Real-Time Locating Systems). This allows for accurate indoor/outdoor localization, even in challenging environments.
- **Robot-to-Robot Localization:**
  Enables localization of a robot relative to another robot by fusing data from odometry, IMU, and RTLS, facilitating cooperative navigation.
- **Robot-to-Human Localization:**
  Localizes a robot relative to a human using odometry, IMU, and RTLS data, supporting human-robot interaction and tracking in real time.

Built on the **romea_core_filtering** library, this package supports asynchronous data fusion from multiple sensor sources, enhancing accuracy and reliability in complex scenarios.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-localisation/refs/heads/main/romea_localisation_public.repos
5. vcs import src < romea_localisation_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to Project Title, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

Project Title is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core Localization library, written by **Jean Laneurit**, was developed within the framework of several research projects, including robot-to-human localization as part of the ANR Baudet ROB project, robot-to-world localization within the ANR Baudet Rob 2 project, and robot-to-robot localization in the context of the ANR Adap2E project. Several individuals contributed scientifically to the development of this library:

**Jean Laneurit**  
**Christophe Debain**
**Roland Chapuis**
**Romuald Aufrere**

## **Contact**

If you have any questions or comments about Romea Core Localisation library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)**.