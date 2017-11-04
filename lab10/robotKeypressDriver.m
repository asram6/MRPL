classdef robotKeypressDriver < handle    
    %robotKeypressDriver Creates a keyboard event handler and then lets     
    % the user drive the robot with the arrow keys.    
    properties(Constant)        
        linVel = 0.02;        
        angVel = 0.06; % 0.006 / 0.1 (for W)    
    end
    
    properties(Access = private)        
        fh=[];    
    end
    
    properties(Access = public)    
    end
    
    methods(Static = true)        
        function drive(robot,vGain)            
            % drive the robot    
            fprintf("in drive\n");
            Vmax = robotKeypressDriver.linVel*vGain;            
            dV = robotKeypressDriver.angVel*robotModel.W*vGain;            
            key = pollKeyboard();  
            
            fprintf("key %d \n", key);
            
            if(key ~= false)                
                if(strcmp(key,'uparrow'))                    
                    disp('up');                 
                    robot.sendVelocity(Vmax,Vmax);     
                elseif(strcmp(key,'downarrow'))      
                    disp('down');             
                    robot.sendVelocity(-Vmax,-Vmax);    
                elseif(strcmp(key,'leftarrow'))         
                    disp('left');                  
                    robot.sendVelocity(Vmax,Vmax+dV);         
                elseif(strcmp(key,'rightarrow'))        
                    disp('right');              
                    robot.sendVelocity(Vmax+dV,Vmax);          
                elseif(strcmp(key,'s'))              
                    disp('stop');             
                    robot.sendVelocity(0.0,0.0);      
                end;       
            end;   
        end
    end
    
    methods(Access = private)
    end
    
    methods(Access = public)   
        function obj = robotKeypressDriver(fh)       
            % create a robotKeypressDriver for the figure handle       
            % normally you call this with gcf for fh        
            fprintf("in constructur 1 \n");
            obj.fh = fh;    
            fprintf("in constructor 2 \n");
            set(fh,'KeyPressFcn',@keyboardEventListener);
            pause(5);
            kh = event.listener(gcf,'KeyPressFcn',@keyboardEventListener);
            pause(5);
            fprintf("in constructor 3 \n");
        end
    end
end



function keyboardEventListener(~,event)
%keyboardEventListener Invoked when a keyboard character is pressed.
    fprintf("in event listener \n ");
    global keypressFrame;
    global keypressDataReady;
    global keypressKey;
    keypressFrame = keypressFrame + 1;
    keypressDataReady = 1;
    keypressKey = event.Key;
end


function res = pollKeyboard()
%pollKeyboard Waits until the callback says there is new data.
%   This routine is useful when you want to be able to capture 
%   and respondto a keypress as soon as it arrives.
%   To use this, execute the following line:
%   kh = event.listener(gcf,'KeyPressFcn',@keyboardEventListener);
%   before calling this function.
    fprintf("in poll keyboard start \n");
    global keypressDataReady;
    global keypressKey;
    keyboardDataReadyLast = keypressDataReady;
    keypressDataReady = 0;
    if(keyboardDataReadyLast)
        res = keypressKey;  
        disp('gotOne');
    else
        res = false;
    end
    fprintf("in poll keyboard end \n");
end

% robot = raspbot("Rasbpot-17");
% pause(3);
% obj = robotKeypressDriver(gcf);
% while true
%     obj.drive(robot, 0.1);
% end

