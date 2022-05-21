%Read point cloud data present in points.las
path = fullfile(toolboxdir('lidar'),'lidardata', ...
    'las','aerialLidarData.laz');
lasReader = lasFileReader('points.las'); 

%Read point cloud data and associated classification point attributes from the LAZ file using the readPointCloud function.
[ptCloud,pointAttributes] = readPointCloud(lasReader,'Attributes','Classification');

%Color the points based on their classification attributes. Reshape the label image into the shape of the point cloud.
labels = label2rgb(pointAttributes.Classification);
colorData = reshape(labels,[],3);

%visualise data
figure
pcshow(ptCloud.Location)