
function a = main(cam,frame,str,basep,date,drive)

    fullfile(pwd, basep)
    %'C:\Users\zifan\OneDrive\Desktop\School\ENGO 500\devkit_raw_data\devkit\2011_09_26_drive_0027_sync')

    a = 1;
    [velo_img, pt] = run_demoVelodyne(append(basep,'\',date,'\',date,'_','drive','_',drive,'_sync'),...
    append(basep ,'\',date),frame,cam);

    writematrix(velo_img, str);

end 
