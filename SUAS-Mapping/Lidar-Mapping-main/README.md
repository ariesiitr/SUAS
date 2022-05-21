LIDAR MAPPING FOR SUAS

Data Set: https://drive.google.com/drive/folders/1S-_bC_d_EQiqbjLI7Wa9-yNQfGdzLVfH?usp=sharing

Introduction to LIDAR:
LIDAR is the abbreviation for Light Detection and Ranging. It uses laser beams to create a 3D representation of the given surrounding environment. A typical lidar sensor emits pulsed light waves into the surrounding environment. These pulses hit surrounding objects and return to the sensor. The sensor uses the time it took for each pulse to return to the sensor to calculate the distance it travelled. This process is repeated millions of times per second to create a precise, real-time 3D map of the environment. An onboard computer (Odriod) can utilize this map for safe navigation.

Required components:
360 TF Lidar
OCB Odroid
Power source
Mapping Camera

Process:
First the process is run as a simulation.
For mapping the required area, we are going to use MATLAB. To do so, firstly we need a dataset for that required area. We can get the live data from Lidar or from online sites.
We use this dataset to plot the point cloud using tools of MATLAB.
Then we perform image processing using the image processing toolbox and generate terrain.

When the point cloud data is plotted after image processing:(LiDAR data already present)

%Read point cloud data present in points.las
path = fullfile(toolboxdir('lidar'),'lidardata', ...
    'las','aerialLidarData.laz');
lasReader = lasFileReader('points.las'); 

%Read point cloud data and associated classification point attributes from the LAZ file using the readPointCloud function.
[ptCloud,pointAttributes] = readPointCloud(lasReader,'Attributes','Classification');

Color the points based on their classification attributes. Reshape the label image into the shape of the point cloud.
labels = label2rgb(pointAttributes.Classification);
colorData = reshape(labels,[],3);

%visualise data
figure
pcshow(ptCloud.Location)

OUTPUT FOR TERRAIN VIEW

When LiDAR data is collected along with drone gps data for map generation and trajectory estimation:
Create appropriate data set from true ground waypoints and LiDAR scan data
Visualise the original scan data
Setup the parameters to filter out similar scan data for faster mapping
Perform mapping from the data set using SLAM by overlapping the different scan being output after going through the above tuning parameters
Generate and refine the generated trajectory using point cloud downsampling
Perform optimization on the generated trajectory using pose graph optimization function in MATLAB
Generate the global map using all the processed point clouds
View the generated map and trajectory in a new window

The above simulation is done on a UAV package present in MATLAB with predefined trajectory and already generated LiDAR (.laz) data

SLAM
Simultaneous localization and mapping (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.
We perform analysis using SLAM in the second part so GPS data becomes important along with LiDAR scans.
OUTPUT FOR THE ABOVE STEPS:

Further Steps:
Perform photogrammetry of the same region to generate a cartographic map and try to overlay the generated LiDAR map and photogrammetric map for greater precision.
Possible softwares to be used: QGis or GIS toolbox in MATLAB

Communication becomes important as this processing may have a requirement of being done after the complete flight of the drone.
