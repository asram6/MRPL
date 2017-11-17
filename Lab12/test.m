classdef test
    methods(Static)
        function vec = matToPoseVec(mat)
            % Convert a homogeneous transform into a vector that can be
            % passed to the contructor for this class.
            x = mat(1,3);
            y = mat(2,3);
            w = atan2(-mat(1,2),mat(1,1));
            vec = [x ; y ; w];
        end


        function mat = bToA(pose)
            % Returns the homogeneous transform that converts coordinates from
            % the b frame to the a frame.

            mat = zeros(3,3);
            x = pose(1); y = pose(2); th = pose(3);

            mat(1,1) =  cos(th); mat(1,2) = -sin(th); mat(1,3) = x;
            mat(2,1) =  sin(th); mat(2,2) =  cos(th); mat(2,3) = y;
            mat(3,1) =  0.0    ; mat(3,2) =  0.0    ; mat(3,3) = 1;
        end


        function mat = aToB(pose)
            % Returns the homogeneous transform that converts coordinates from
            % the a frame to the b frame.

            bTa = test.bToA(pose);

            mat = bTa^-1;
        end


        function absPose = relToAbs()
            poseR = [.9,.3,0];
            relPose = [-.3,.3,pi/2];
            absPose = test.matToPoseVec(test.bToA(poseR)*test.bToA(relPose));
        end
    
    end
end

