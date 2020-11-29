k = 100; g = 9.8; I = 1; J=1; m=1; L=1;

b1=1; b2=0;

% Rule 1
A1 = [              0   0 1 0;
                    0    0 0 1;
      (-k-m*g*L*b1)/I  k/I 0 0;
                  k/J -k/J 0 0];

B1 = [0 0 0 1]';

% Rule 2
A2 = [              0   0 1 0;
                    0    0 0 1;
      (-k-m*g*L*b2)/I  k/I 0 0;
                  k/J -k/J 0 0];
            
B2 = [0 0 0 1]';

setlmis([])

n = 4;                    
X = lmivar(1, [n, 1]);      
Y1 = lmivar(2, [1, n]);
Y2 = lmivar(2, [1, n]);

% LMI condition 1
lmiterm([1 1 1 X], A1, 1, 's')      % A1*X + X*A1'
lmiterm([1 1 1 Y1], -B1, 1, 's')    % -B1*Y1 - Y1'*B1'
% LMI condition 2
lmiterm([2 1 1 X], A2, 1, 's')      % A2*X + X*A2'
lmiterm([2 1 1 Y1], -B2, 1, 's')    % -B2*Y1 - Y1'*B2'
% LMI condition 3
lmiterm([3 1 1 X], A2, 1, 's')      % A1*X + X*A1'
lmiterm([3 1 1 Y2], -B1, 1, 's')    % -B1*Y2 - Y2'*B1'
% LMI condition 4
lmiterm([4 1 1 X], A2, 1, 's')      % A2*X + X*A2'
lmiterm([4 1 1 Y2], -B2, 1, 's')    % -B2*Y2 - Y2'*B2'
% LMI condition 5
lmiterm([-5 1 1 X], 1, 1)   % specify right side

lmi_sys = getlmis;

[tmin, xfeas] = feasp(lmi_sys);

if tmin < 0
    Xs = dec2mat(lmi_sys, xfeas, X);
    Ys1 = dec2mat(lmi_sys, xfeas, Y1);
    Ys2 = dec2mat(lmi_sys, xfeas, Y2);
end

F1 = Ys1 * inv(Xs);
F2 = Ys2 * inv(Xs);
P = inv(Xs);

x = [90 60 0 0]';
u = 0;
for i=1:899  
    % model
    ddot_x3=(k*(x(2)-x(1))-m*g*L*sind(x(1)))/I;
    
    ddot_x4=(u-k*(x(2)-x(1)))/J;
    
    x(3) = ddot_x3;
    x(1) = x(3)*0.01-180;
    
    x(4) = ddot_x4;
    x(2) = x(4)*0.01-180;
    
    % controller
    u = -sind(x(1))/x(1)*(F1*x)-(x(1)-sind(x(1)))/x(1)*(F2*x);    
    check_input(:,i) = u;
    check_state(:,i) = x;
    
end

Initial_angle1 = 90;
Initial_angle2 = 60;