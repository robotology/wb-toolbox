classdef PIDConfiguration < handle
    % PIDCONFIGURATION Class to store a set of PID config for many joints.
    %   PIDCONFIGURATION(WBToolbox.PID('jointName', P, I, D)) adds a new PID
    %   for joint 'jointName'.
    %
    % PIDCONFIGURATION Properties:
    %   P - Proportional gains
    %   I - Integral gains
    %   D - Derivative gains
    %   jointList - List of joints
    %
    % PIDCONFIGURATION Methods:
    %   addPID - Add a new joint PID
    %   removePID - Remove a stored joint PID
    %
    % See also: WBTOOLBOX.PID

    % Read-Only properties
    properties (SetAccess = private)
        P         double
        I         double
        D         double
        jointList cell
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

        function value = getSimulinkParameters(obj)
            if isempty(obj.P) || isempty(obj.I) || isempty(obj.D) || ...
                    isempty(obj.jointList)
                error('Trying to get parameters from an empty object')
            end
            value = struct();
            value.P = obj.P;
            value.I = obj.I;
            value.D = obj.D;
            value.jointList = obj.jointList;
        end
    end

end
