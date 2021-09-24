clear;
clc;
close all;
robot = importrobot('universalUR10.urdf',"MeshPath",["../asset/ur_description/ur10/collision","../asset/ur_description/ur10/visual"]);
show(robot,'visuals','off','collision','on')