function encoderEventListener(handle, event)
    %EncoderEventListener Invoked when new encoder data arrives. 
    % A MATLAB event listener for the Robot. Invoked when
    % encoder data arrives.
    fprintf("HERE");
    fprintf('%d', 1);
    fprintf('%d', handle);
    fprintf('%d', event);
end