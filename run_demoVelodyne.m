function [velo_img, pt] = run_demoVelodyne (base_dir,calib_dir,f,c)
% KITTI RAW DATA DEVELOPMENT KIT
% 
% Demonstrates projection of the velodyne points into the image plane
%
% Input arguments:
% base_dir .... absolute path to sequence base directory (ends with _sync)
% calib_dir ... absolute path to directory that contains calibration files

% clear and close everything
close all; dbstop error; clc;
disp('======= KITTI DevKit Demo =======');

% options (modify this to select your sequence)
% if nargin<1
%   base_dir  = '/mn/karlsruhe_dataset/2011_09_26/2011_09_26_drive_0009_sync';
% end
% if nargin<2
%   calib_dir = '/mnt/karlsruhe_dataset/2011_09_26';
% end
cam       = c; % 0-based index
frame     = f; % 0-based index

% load calibration
calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));

% compute projection matrix velodyne->image plane
R_cam_to_rect = eye(4);
R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
P_velo_to_img = calib.P_rect{cam+1}*R_cam_to_rect*Tr_velo_to_cam;

% load and display image
img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
imshow(img); hold on;

% load velodyne points
fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin',base_dir,frame),'rb');
velo = fread(fid,[4 inf],'single')';
velo = velo(1:5:end,:); % remove every 5th point for display speed
fclose(fid);



% remove all points behind image plane (approximation)
idx = velo(:,1)<4;
velo(idx,:) = [];
test=[];

%---GroundSeg----%
% pt = velo(:,1:end-1);
% color = velo(:,end);
% ptCloud = pointCloud(pt,'Intensity',color);
% hResolution=512;
% params =lidarParameters ("HDL64E", hResolution);
% ptCloud = pcorganize(ptCloud,params);
% groundPtsIdx = segmentGroundFromLidarData(ptCloud);
% groundPtCloud = select(ptCloud,groundPtsIdx);
%---End----%




% figure
% pt = test(:,1:end-1);
% color = test(:,end);
% ptCloud = pointCloud(pt,'Intensity',color);
% pcshow(ptCloud);

img =[];
% plot points
cols = jet;
count =1;

% project to image plane (exclude luminance)
velo_img = project(velo(:,1:3),P_velo_to_img);


% for i=1:size(velo_img,1)
%   if(velo(i,3)<-1.43)
%        test(count,:) = velo(i,:);
%        img(count,:) = velo_img(i,:);
%        count=count+1;
%   end    
% end

% check = mode(test);
% for i=1:size(velo_img,1)    
%      col_idx = 5;  
%   if(velo(i,3)<-1.43&& velo(i,3) > check(1,3)*1.1 && velo(i,3) <check(1,3)*0.95)
%        test(count,:) = velo(i,:);
%        img(count,:) = velo_img(i,:);
%        count=count+1;
%        plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',1,'MarkerSize',4,'Color',cols(col_idx,:));
%   end     
% end

pt = velo(:,1:end-1);
color = velo(:,end);
ptCloud = pointCloud(pt,'Intensity',color);
hResolution=512;
params =lidarParameters ("HDL64E", hResolution);
ptCloud = pcorganize(ptCloud,params);

groundPtsIdx=segmentGroundFromLidarData(ptCloud);
groundPtCloud = select(ptCloud,groundPtsIdx);




maxd = 0.05;
normalv =[0 0 1];
maxAngularDistance = 15;

[model1,inlierIdx,outlierIdx] = pcfitplane(groundPtCloud,maxd,normalv,maxAngularDistance);
road = select(groundPtCloud,inlierIdx);



velo_img = project(road.Location,P_velo_to_img);

% && velo(i,3) > check(1,3)*1.5 && velo(i,3) <check(1,3)*0.5 &&velo(i,4) <= check(1,4)*1.5 &&velo(i,4) >= check(1,4)*0.5
for i=1:size(velo_img,1)
    col_idx = 5;      
    plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',1,'MarkerSize',4,'Color',cols(col_idx,:));
     
end


figure;
pcshow(ptCloud);
title("Unorganized Lidar Point Cloud");
figure;
pcshow(groundPtCloud);
title('Segmented Ground Lidar Point Cloud');
figure;
pcshow(road);
title('Idenitified Best Plan-fit Road Lidar Point Cloud');