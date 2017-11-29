classdef Configuration < matlab.mixin.Copyable
    %CONFIGURATION Summary of this class goes here
    %   Detailed explanation goes here

    % PROPERTIES
    % ==========

    properties
        % Robot state
        RobotName          (1,:) char
        UrdfFile           (1,:) char
        ControlledJoints   (1,:) cell
        ControlBoardsNames (1,:) cell
        LocalName          (1,:) char
        % Other Variables
        GravityVector      (1,3) double = [0, 0, -9.81]
    end

    properties (Dependent)
        ValidConfiguration (1,1) logical
    end

    properties (Hidden, Dependent)
        SimulinkParameters (1,1) struct
    end

    % METHODS
    % =======

    methods

        % SET METHODS
        % -----------

        % Dependent properties
        % --------------------

        function set.ValidConfiguration(~,~)
            error(['ValidConfiguration is set to 1 automatically ', ...
                'when the configuration is valid.']);
        end

        function set.SimulinkParameters(~,~)
            error(['SimulinkParameters is created automatically ', ...
                'when the configuration is valid.']);
        end

        % Other properties
        % ----------------

        function set.ControlledJoints(obj, value)
            if (~iscellstr(value))
                error(['ControlledJoints must be a cell array of ', ...
                    'charachter vectors']);
            end
            obj.ControlledJoints = value;
        end

        function set.ControlBoardsNames(obj, value)
            if (~iscellstr(value))
                error(['ControlBoardsNames must be a cell array of ', ...
                    'charachter vectors']);
            end
            obj.ControlBoardsNames = value;
        end

        % GET METHODS
        % -----------

        % Dependent properties
        % --------------------

        function value = get.ValidConfiguration(obj)
            % Check if all the properties have been set
            value = ...
                ~isempty(obj.RobotName) && ...
                ~isempty(obj.UrdfFile)  && ...
                ~isempty(obj.ControlledJoints)   && ...
                ~isempty(obj.ControlBoardsNames) && ...
                ~isempty(obj.LocalName) && ...
                ~isequal(obj.GravityVector,[0, 0, 0]);
        end

        function value = get.SimulinkParameters(obj)
            % Create and populate the struct
            value = struct();
            value.RobotName = obj.RobotName;
            value.UrdfFile  = obj.UrdfFile;
            value.LocalName = obj.LocalName;
            value.ControlledJoints   = obj.ControlledJoints;
            value.ControlBoardsNames = obj.ControlBoardsNames;
            value.GravityVector = obj.GravityVector;
        end

        % Other properties
        % ----------------

        % OTHER METHODS
        % =============

        function value = getSimulinkParameters(obj)
            if (obj.ValidConfiguration)
                value = obj.SimulinkParameters;
            else
                error('The configuration is not complete');
            end
        end
    end

    methods(Static)

        function cellArraySerialized = serializeCellArray1D(cellArray)
            [m,n] = size(cellArray);
            if (m>1 && n>1)
                error('The input cell must be 1D');
            end
            cellArraySerialized = '{';
            for i=1:length(cellArray)
                cellArraySerialized = strcat(cellArraySerialized,'''',cellArray{i},'''');
                if i ~= length(cellArray)
                    cellArraySerialized = strcat(cellArraySerialized,',');
                end
            end
            cellArraySerialized = strcat(cellArraySerialized,'}');
        end

        function vectorSerialized = serializeVector1D(vector)
            [m,n] = size(vector);
            if (m>1 && n>1)
                error('The input vector must be 1D');
            end
            vectorSerialized = '[';
            for i=1:length(vector)
                vectorSerialized = strcat(vectorSerialized,num2str(vector(i)));
                if i ~= length(vector)
                    vectorSerialized = strcat(vectorSerialized,',');
                end
            end
            vectorSerialized = strcat(vectorSerialized,']');
        end

    end
end
