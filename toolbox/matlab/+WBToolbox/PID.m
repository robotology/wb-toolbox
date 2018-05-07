% Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
% This software may be modified and distributed under the terms of the
% GNU Lesser General Public License v2.1 or any later version.

classdef PID < handle
    %PID Class to store pid parameters for a single joint.
    %   PID('jointName') creates an object for the joint 'jointName' with
    %   all pid gains set to 0.
    %
    %   PID('jointName', P, I, D) creates an object for the joint 'jointName'
    %   initializing the gains respectively to P, I, D.
    %
    % PID Properties:
    %   P - Proportional gain
    %   I - Integral gain
    %   D - Derivative gain
    %   joint - Reference joint
    %
    % See also: WBTOOLBOX.PIDCONFIGURATION

    properties
        P     double = 0
        I     double = 0
        D     double = 0
        joint char
    end

    methods
        %         function obj = PID(jointName)
        %             obj.joint = jointName;
        %         end

        function obj = PID(jointName, P, I, D)
            if (nargin == 4)
                obj.joint = jointName;
                obj.P = P;
                obj.I = I;
                obj.D = D;
            elseif (nargin == 1)
                obj.joint = jointName;
            else
                error(...
                    [...
                    'Wrong number of arguments: ', ...
                    'You can either specify 1 argument (joint) ', ...
                    'or 4 arguments (joint, P, I, D).',...
                    ]...
                    );
            end
        end

    end
end
