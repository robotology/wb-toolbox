classdef WBTPIDConfig < handle
    % WBTPIDCONFIG Class to store a set of PID config for many joints.
    %   WBTPIDCONFIG(WBToolbox.PID('jointName', P, I, D)) adds a new PID
    %   for joint 'jointName'.
    %
    % WBTPIDCONFIG Properties:
    %   P - Proportional gains
    %   I - Integral gains
    %   D - Derivative gains
    %   jointList - List of joints
    %
    % WBTPIDCONFIG Methods:
    %   addPID - Add a new joint PID
    %   removePID - Remove a stored joint PID
    %
    % See also: WBTOOLBOX.PID
    
    % Read-Only properties
    properties (SetAccess = private)
        P         (1,:) double
        I         (1,:) double
        D         (1,:) double
        jointList (1,:) cell
    end
    
    methods
        function addPID(obj, pid)
            % Add
            if ~isa(pid,'WBToolbox.PID')
                error('The argument must be a WBToolbox.PID object.')
            end
            
            matches = find(strcmp(obj.jointList, pid.joint), 1);
            if ~isempty(matches)
                error('The PID''s joint has been already processed.')
            end
            
            obj.jointList{1, end+1} = pid.joint;
            obj.P(1, end+1) = pid.P;
            obj.I(1, end+1) = pid.I;
            obj.D(1, end+1) = pid.D;
        end
        
        function removePID(obj, jointName)
            if ~ischar(jointName)
                error('The argument must be a char containing the joint name.')
            end
            
            idx = find(strcmp(obj.jointList, jointName), 1);
            
            if isempty(idx)
                error('The specified joint has no configuration stored')
            end
            
            obj.jointList(:,idx) = [];
            obj.P(:,idx) = [];
            obj.I(:,idx) = [];
            obj.D(:,idx) = [];
        end
    end
    
end

