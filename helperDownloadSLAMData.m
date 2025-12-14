function vioData = helperDownloadSLAMData()
% helperDownloadData downloads the data set from the specified URL to the
% specified output folder.

    % vioDataTarFile =  matlab.internal.examples.downloadSupportFile(...
    %     'shared_nav_vision/data','BlackbirdVIOData.tar');  
    % 
    % % Extract the file.
    % outputFolder = fileparts(vioDataTarFile);
    % if (~exist(fullfile(outputFolder,"BlackbirdVIOData"),"dir"))
    %     untar(vioDataTarFile,outputFolder);
    % end

    % vioData = load(fullfile(outputFolder,"BlackbirdVIOData","data.mat"));

    % vioData = load("./BlackbirdVIOData/att_data_f5c02_delay20_continue5.mat"); % 攻击数据
    vioData = load("./BlackbirdVIOData/data.mat"); % 原始数据
end