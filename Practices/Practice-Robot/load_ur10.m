floor = collisionBox(1, 1, 0.01);
tabletop1 = collisionBox(0.4,1,0.02);
tabletop1.Pose = trvec2tform([0.3,0,0.6]);
tabletop2 = collisionBox(0.6,0.2,0.02);
tabletop2.Pose = trvec2tform([-0.2,0.4,0.5]);
can = collisionCylinder(0.03,0.16);
can.Pose = trvec2tform([0.3,0.0,0.7]);trvec2tform([0.3,0,0.6])