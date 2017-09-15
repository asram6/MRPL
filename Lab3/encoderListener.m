classdef encoderListener
   properties
      PropertyName
   end
   methods(Static)
    function encoderEventListener(handle, event)
        %EncoderEventListener Invoked when new encoder data arrives. 
        % A MATLAB event listener for the Robot. Invoked when
        % encoder data arrives.
        fprintf("HERE");
        fprintf('%d', 1);
        fprintf('%d', handle);
        fprintf('%d', event);
    end
    function main()
        robot = raspbot('Raspbot-09');
        robot.encoders.NewMessageFcn = @encoderEventListener;
        pause(10)
        robot.shutdown();
    end
   end
end 