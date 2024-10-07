## Romea Core Localisation Library

This library provides robust implementations of three key localization algorithms:

- **Robot-to-World Localization:**
  Localizes a robot within the global environment using sensor data from odometry, IMU, GPS, LIDAR (coming soon), RADAR, and RTLS (Real-Time Locating Systems). This allows for accurate indoor/outdoor localization, even in challenging environments.
- **Robot-to-Robot Localization:**
  Enables localization of a robot relative to another robot by fusing data from odometry, IMU, and RTLS, facilitating cooperative navigation.
- **Robot-to-Human Localization:**
  Localizes a robot relative to a human using odometry, IMU, and RTLS data, supporting human-robot interaction and tracking in real time.

Built on the **romea_core_filtering** library, this package supports asynchronous data fusion from multiple sensor sources, enhancing accuracy and reliability in complex scenarios.