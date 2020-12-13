# Extended Kalman Filter Project Starter Code

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


<img src="Capture1.JPG" alt="image"/>

<h1> Overview </h1>
<p>  The project focusses using the sensor fusion technique to estimate the position of vehicle (in this case a bicycle) and also the velocity of the vechile in both x and y directions. The sensors used here are RADAR and LiDAR. RADAR sensors are useful for measuring the velocity and direction of the movement of the object. Similarily LiDAR is used to measure the x and y position of the object. So we fuse both these sensors to get the estimate of x & y position and x & y direction velocity. The sensors have noise in their readings and so we need to process it to get accurate position and velocity which is the key for the self driving car. Here we use the Kalman filter approch to provide estimation of the moving object. <p>
 
<h1> Sensor data processing technique </h1>

<p> The LiDAR sensor provides a measurement of the position of the object in x and y dimensions. These values are all linear in nature. So the Kalman filter is directly used to process the sensor data. The Kalman filter uses Gaussian distribution and provides and output in a Gaussian Curve with mean and SD.</p>

<h3> Kalman Filter </h3>

