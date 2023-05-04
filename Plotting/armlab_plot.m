%% 
bag = rosbag('armlab.bag')
bagInfo = rosbag('info', 'armlab.bag')

rosbag info 'armlab.bag';
%%
bagselect = rosbag('armlab.bag');
bagselect2 = select(bagselect, 'Topic', '/rx200/joint_states');
msgStructs = readMessages(bagselect2, 'DataFormat', 'struct');
msgStructs{1}.Position

% Joint states
ln = int32(length(msgStructs));
j1 = zeros(ln,1)
j2 = zeros(ln,1);
j3 = zeros(ln,1);
j4 = zeros(ln,1);
j5 = zeros(ln,1);
j6 = zeros(ln,1);
j7 = zeros(ln,1);
j8 = zeros(ln,1);

i = 1;
for j = 1:length(msgStructs)
%    if(mod(j,2) == 0)
        j1(i) = msgStructs{i}.Position(1);
        %msgStructs{i}.Position(1);
        j2(i) = msgStructs{i}.Position(2);
        j3(i) = msgStructs{i}.Position(3);
        j4(i) = msgStructs{i}.Position(4);
        j5(i) = msgStructs{i}.Position(5);
        j6(i) = msgStructs{i}.Position(6);
        j7(i) = msgStructs{i}.Position(7);
        j8(i) = msgStructs{i}.Position(8);
        i = i + 1;
        %x_y(i) = fk(j1,j2...
        %    end
end
plot(j1)
hold on
plot(j2)
plot(j3)
plot(j4)
plot(j5)
plot(j6)
plot(j7)
plot(j8)
legend("Waist","Shoulder","Elbow","Wrist Angle","Wrist Rotate","Gripper","Gripper Left Finger","Gripper Right Finger")
hold off
