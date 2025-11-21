a1 = 0.12;      % link 1 (m)
a2 = 0.118;     % link 2 (m)
a3 = 0.083;     % cổ tay + bút (m)
a4 = 0.05;     % link cố định vuông góc, 4.5 cm (m)

armRobot4DOF = ( ...
    ETS3.Rz('q1') * ...    % khớp 1: đế
    ETS3.Ry('q2') * ...    % khớp 2: vai
    ETS3.Tz(a1) * ...      % link 1
    ETS3.Ry('q3') * ...    % khớp 3: khuỷu
    ETS3.Tz(a2) * ...      % link 2
    ETS3.Ry('q4') * ...    % khớp 4: cổ tay
    ETS3.Tz(a3) * ...      % cổ tay + bút
    ETS3.Tx(a4));          % ★ link cố định vuông góc theo trục x
figure;
armRobot4DOF.teach;
