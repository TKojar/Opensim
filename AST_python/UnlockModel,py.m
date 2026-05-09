function [modelUnlocked]= UnlockModel(ModelFolder,ModelFile)
% Function that unlocks all the coordinates of a OpenSim MSK model
% Author: Andrea Di Pietro, 17/02/2023 , University of Pisa
import org.opensim.modeling.*;
BaseModel=Model(fullfile(ModelFolder,ModelFile));
num=0;
for u=0:BaseModel.getJointSet.getSize-1 % for every joint
    for v=0:BaseModel.getJointSet.get(u).numCoordinates-1 % get every coordinate from every joint
       BaseModel.getJointSet.get(u).get_coordinates(v).set_locked(0)% check every coordinate
       num=num+1;
    end
end
modelUnlocked=BaseModel;
modelUnlocked.print(fullfile(ModelFolder,[ModelFile(1:end-5) '_Unlocked.osim']));
end