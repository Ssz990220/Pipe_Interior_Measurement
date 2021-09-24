clear;
clc;
close all;
robot = importrobot('universalUR10.urdf',"MeshPath",["../ur_description/ur10/collision","/ur_description/ur10/visual"]);
show(robot,'visuals','off','collision','on')