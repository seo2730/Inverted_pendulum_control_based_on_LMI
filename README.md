# Inverted pendulum control based on LMI
T-S model by MATLAB

# Model
![image](https://user-images.githubusercontent.com/42115807/103292028-592dee00-4a30-11eb-9a38-30b688bbc774.png)<br>

# State-space Equation
![image](https://user-images.githubusercontent.com/42115807/103292068-76fb5300-4a30-11eb-9067-08c0d9824b1e.png)<br>

# Fuzzy variable
![image](https://user-images.githubusercontent.com/42115807/103292119-8aa6b980-4a30-11eb-8886-e78d8b674ee7.png)

# Membership functions
![image](https://user-images.githubusercontent.com/42115807/103292201-b4f87700-4a30-11eb-998e-62f7c3dde737.png)

# Fuzzy Rules
#### Rule 1
![image](https://user-images.githubusercontent.com/42115807/103292324-e8d39c80-4a30-11eb-97b1-5be7339e9e29.png)

#### Rule 2
![image](https://user-images.githubusercontent.com/42115807/103292357-f557f500-4a30-11eb-8c89-d43f758e87b0.png)

# Matlab code
##### You need to install LMI toolbox.<br>
<br>
- parameter<br>

    k = 100; g = 9.8; I = 1; J=1; m=1; L=1;
    b1=1; b2=0;

<br>
- LMI setting<br>

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

- Initial condition<br>

    Initial_angle1 = 90;
    Initial_angle2 = 60;

# Simulink
Block diagram<br>
![image](https://user-images.githubusercontent.com/42115807/103292510-4536bc00-4a31-11eb-8448-40ff1cc22820.png)<br>
180 is input value.<br>
<br>
Fuzzy controller<br>
![image](https://user-images.githubusercontent.com/42115807/103293283-e6724200-4a32-11eb-8cae-6bcdc0c45827.png)<br>
<br>
Fuzzy model<br>
![image](https://user-images.githubusercontent.com/42115807/103293319-fab63f00-4a32-11eb-9f86-43aae1490fc2.png)<br>
<br>
# Result
![image](https://user-images.githubusercontent.com/42115807/103292936-3ac8f200-4a32-11eb-9439-d09ec1fc9098.png)
