function keyboardEventListener111(~,event)
%keyboardEventListener Invoked when a keyboard character is pressed.
    fprintf("in event listener\n");
    global keypressFrame;
    global keypressDataReady;
    global keypressKey;
    keypressFrame = keypressFrame + 1;
    keypressDataReady = 1;
    keypressKey = event.Key;
end