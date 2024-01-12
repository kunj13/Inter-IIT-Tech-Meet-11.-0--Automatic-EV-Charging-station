clc;
clear all;
d1 = 1.4;
d2 = 2.2;
d3 = -1.6;
z1 = 1.4;
x1 = 2.2;
y1 = -1.6;
z_mat = zeros(50,1);
x_mat = zeros(50,1);
y_mat = zeros(50,1);
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','prismatic');
jnt1.JointAxis = [0,0,-1];
jnt1.PositionLimits = [0 1.4];
tform = trvec2tform([0, 0, d1]); 
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base')
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','prismatic');
jnt2.PositionLimits = [-2,2];
tform2 = trvec2tform([d2, 0, 0]);
jnt2.JointAxis = [-1 0 0];
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','prismatic');
jnt3.PositionLimits=[-2.5 0];
jnt3.JointAxis = [0 -1 0];
tform3 = trvec2tform([0,d3,0]);
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
tform4 = trvec2tform([0,-0.05,0]);
setFixedTransform(jnt4,tform4);
body4.Joint = jnt4;
addBody(robot,body4,'body3');
gik = generalizedInverseKinematics();
gik.RigidBodyTree = robot;
gik.ConstraintInputs = {'position'};
posTgt = constraintPositionTarget('body3');
posTgt.TargetPosition = [2 -1.4 1.2];
q0 = homeConfiguration(robot);
[q,solutionInfo] = gik(q0,posTgt);
plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[0.5 0.5 0.5 0.5 0.5],'r')
hold on;
plot3([2 2 2 2],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
hold on;
plot3([1 1 1 1],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
hold on;
plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[1.2 1.2 1.2 1.2 1.2],'r');
hold on;
show(robot)
hold on;
show(robot,q)
title(['Solver status: ' solutionInfo.Status])
zlim([0 2])
ylim([-3 3])
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
figure;
for i=1:50
    if(q0(1).JointPosition < (q(1).JointPosition/abs(q(1).JointPosition))*q(1).JointPosition)
        q0(1).JointPosition = q0(1).JointPosition + (q(1).JointPosition/abs(q(1).JointPosition))*0.01;
        z_mat(i) = z1;
        z1 = z1 - (q(1).JointPosition/abs(q(1).JointPosition))*0.01;
        hold off;
        plot3([0 0],[0 0],[0 z1],'g-o');
        hold on;
        plot3([0 2.2],[0 0],[z1 z1],'g-o');
        hold on;
        plot3([2.2 2.2],[0 -1.6],[z1 z1],'g-o');
        hold on;
        plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[0.5 0.5 0.5 0.5 0.5],'r')
        hold on;
        plot3([2 2 2 2],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
        hold on;
        plot3([1 1 1 1],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
        hold on;
        plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[1.2 1.2 1.2 1.2 1.2],'r');
        pause(0.5);
    end
end
for j=1:50
    if(q0(2).JointPosition < (q(2).JointPosition/abs(q(2).JointPosition))*q(2).JointPosition)
        q0(2).JointPosition = q0(2).JointPosition + (q(2).JointPosition/abs(q(2).JointPosition))*0.01;
        x_mat(j) = x1;
        x1 = x1 - (q(2).JointPosition/abs(q(2).JointPosition))*0.01;
        hold off;
        plot3([0 0],[0 0],[0 z1],'g-o');
        hold on;
        plot3([0 x1],[0 0],[z1 z1],'g-o');
        hold on;
        plot3([x1 x1],[0 -1.6],[z1 z1],'g-o');
        hold on;
        plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[0.5 0.5 0.5 0.5 0.5],'r')
        hold on;
        plot3([2 2 2 2],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
        hold on;
        plot3([1 1 1 1],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
        hold on;
        plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[1.2 1.2 1.2 1.2 1.2],'r');
        pause(0.5);
    end
end
for k=1:50
    if(q0(3).JointPosition > (-q(3).JointPosition/abs(q(3).JointPosition))*q(3).JointPosition)
        q0(3).JointPosition = q0(3).JointPosition - (-q(3).JointPosition/abs(q(3).JointPosition))*0.01;
        y_mat(k) = y1;
        y1 = y1 + (-q(3).JointPosition/abs(q(3).JointPosition))*0.01;
        hold off;
        plot3([0 0],[0 0],[0 z1],'g-o');
        hold on;
        plot3([0 x1],[0 0],[z1 z1],'g-o');
        hold on;
        plot3([x1 x1],[0 y1],[z1 z1],'g-o');
        hold on;
        plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[0.5 0.5 0.5 0.5 0.5],'r')
        hold on;
        plot3([2 2 2 2],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
        hold on;
        plot3([1 1 1 1],[-1 -1 -1.4 -1.4],[0.5 1.2 1.2 0.5],'r')
        hold on;
        plot3([2 2 1 1 2],[-1 -1.4 -1.4 -1 -1],[1.2 1.2 1.2 1.2 1.2],'r');
        pause(0.5);
    end
end
figure;