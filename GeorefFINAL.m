%This application procedes the automatic georeferencing of an image pixel point in the navigation frame, or in the ENU Frame 
display('This algorithm georeferences an image pixel point in the navigation frame, or in the ENU Frame, from its coordinates in the Image Frame');
%1st step: rotation from Image to Camera frames
display('First step: coordinates transformation from the Image Frame to the Camera Frame')
%Input the camera intrinsic parameters
display('Please, enter with the camera intrinsic parameters:');
%considering the scale factors sx and sy, fx and fy must be calculated
%f = input ('Enter with "f":  ');
%sx = input ('Enter with "sx":  ');
%fx = sx*f
%sy = input ('Enter with "sy":  ');
%fy = sy*f
fx = input ('Enter with "fx":  ');
fy = input ('Enter with "fy":  ');
cx = input ('Enter with "cx":  ');
cy = input ('Enter with "cy":  ');
%factor gama is usually zero
%gama = input ('Enter with "gama":  ');
%s = input ('Enter the scale factor "s":  ');
%Intrinsic camera parameter matrix K
display('The camera Intrinsic parameter matrix:');
K = [fx 0 cx; 0 fy cy; 0 0 1];
display(K);
%Obtaining the pixel corrdinates in the camera frame, "Pc", from the pixel coordinates
%in the image frame, "Pi"
display('Please, enter with the pixel coordinates:');
u = input ('Enter with u:  ');
v = input ('Enter with v:  ');
%Pixel position matrix in the image frame, "Pi"
Pi = [u; v; 1];
display(Pi);
%Pixel position matrix in the camera frame, "PpimeC"
PprimeC = K\Pi;
display('The pixel position matrix in the camera frame, "PpimeC", is given by:  ');
display(PprimeC);
%2nd step: rotation from the camera frame to the gimbal frame
display('Second step: coordinates transformation from the Camera Frame to the Gimbal Frame');
display('The rotation matrix from Camera FRame to the Gimbal Frame, Rcg is given by:');
%Rotation matrix from the camera frame to the gimbal frame (is a default,
%most commonly used)
Rcg = [0 0 1; 1 0 0; 0 1 0];
display(Rcg);
%translation matrix Tcg between the camera and gimbal positions
display('Please, enter with the coordinates of the translation matrix, Tcg, between camera and gimbal');
Tcgx = input('Please, enter with x coordinate:   ');
Tcgy = input('Please, enter with y coordinate:   ');
Tcgz = input('Please, enter with z coordinate:   ');
Tcg = [Tcgx; Tcgy; Tcgz];
display('The translation matrix Tcg between the camera and gimbal positions is given by:   ');
display(Tcg);
%The prime pixel position in the gimbal frame, PprimeG
PprimeG = Rcg*PprimeC + Tcg;
display('The prime pixel position in the Gimbal Frame, PprimeG, is given by:   ');
display(PprimeG);
%3rd STEP: rotation from the gimbal frame to the UAS frame
%Obtaining the Direction Cosine Matrix, Ruasg for matrix rotation from the
%UAS frame to the Gimbal frame, beacuse the Euler angles are given in
%relation to the UAS frame
display('Third step: coordinates transformation from the Gimbal Frame to the UAS Frame')
display('Please, enter with the rotation angles:   ');
yawuasg = input ('Enter with yaw angle:  ');
pitchuasg = input ('Enter with pitch angle:  ');
rolluasg = input ('Enter with roll angle:  ');
%Obtaining the DCM matrix
Ruasg = angle2dcm(yawuasg,pitchuasg,rolluasg);
Rguas = transpose(Ruasg);
display('The DCM-Direction Cosine Matrix, Rguas, between the Gimbal Frame and the UAS Frame is given by:  ');
display(Rguas);
%translation matrix Tcg between the camera and gimbal positions
display('Please, enter with the coordinates of the translation matrix, Tguas, between Gimbal and UAS frames');
Tguasx = input('Please, enter with x coordinate:   ');
Tguasy = input('Please, enter with y coordinate:   ');
Tguasz = input('Please, enter with z coordinate:   ');
Tguas = [Tguasx; Tguasy; Tguasz];
display('The translation matrix Tcg between the Gimbal and UAS positions, Tguas, is given by:   ')
display(Tguas);
%pixel position in the UAS frame
PprimeUAS = Rguas*PprimeG + Tguas;
display('The prime pixel position in the UAS Frame, PprimeUAS, is given by:   ');
display(PprimeUAS);
%4th STEP: coordinates transformation from the UAS frame to the NED frame
%Obtaining the Direction Cosine Matrix, Ruasned for matrix rotation from
%the UAS
%frame to the NED frame
display('Fourth step: coordinates transformation from the UAS Frame to the NED Frame')
display('Please, enter with the rotation angles:   ');
yawneduas = input ('Enter with yaw angle:  ');
pitchneduas = input ('Enter with pitch angle:  ');
rollneduas = input ('Enter with roll angle:  ');
%Obtaining the DCM matrix
Rneduas = angle2dcm(yawneduas,pitchneduas, rollneduas);
Ruasned = transpose(Rneduas);
display('The DCM-Direction Cosine Matrix, Ruasned, between the UAS and the NED frames is given by:  ');
display(Ruasned);
%translation matrix Tuasned between the UAS and NED positions
display('Please, enter with the coordinates of the translation matrix, Tuasned, between UAS and NED');
Tuasnedx = input('Please, enter with x coordinate:   ');
Tuasnedy = input('Please, enter with y coordinate:   ');
Tuasnedz = input('Please, enter with z coordinate:   ');
Tuasned = [Tuasnedx; Tuasnedy; Tuasnedz];
display('The translation matrix between the UAS and NED positions, Tuasned, is given by:   ');
display(Tuasned);
%pixel position in the NED frame
PprimeNED = Ruasned*PprimeUAS + Tuasned;
display('The prime pixel position in the NED Frame, PprimeNED, is given by:   ');
display(PprimeNED);
%5th STEP: coordinates transformation from the NED frame to the ENU frame
display('Fifth step: coordinates transformation from the NED Frame to the ENU Frame');
display('The rotation matrix from NED FRame to the ENU Frame, Rnedenu, is given by:  ');
%Rotation matrix from the NED frame to the ENU frame 
Rnedenu = [0 1 0; 1 0 0; 0 0 -1];
display(Rnedenu);
%translation matrix Tnedenu between the NED and ENU positions
display('Please, enter with the coordinates of the translation matrix, Tnedenu, between NED and ENU frames');
Tnedenux = input('Please, enter with x coordinate:   ');
Tnedenuy = input('Please, enter with y coordinate:   ');
Tnedenuz = input('Please, enter with z coordinate:   ');
Tnedenu = [Tnedenux; Tnedenuy; Tnedenuz];
display('The translation matrix Tnedenu between the NED and ENU positions is given by:   ');
display(Tnedenu);
%The prime pixel position in the ENU frame, PprimeENU
PprimeENU = Rnedenu*PprimeNED + Tnedenu;
display('The prime pixel position in the ENU frame, PprimeENU, is given by:   ');
display(PprimeENU);
%6th STEP: determination of the real pixel coordinates in the ENU frame
display('Sixth step: determination of the real pixel coordinates in the ENU frame');
zENU = input('Please, enter with the target altitude:   ');
%Following the precedent equations, the complete translation matrix will be
%given by:
% Tall = Rnedenu*Ruasned*Rguas*Tcg+Rnedenu*Ruasned*Tguas+Rnedenu*Tuasned+Tnedenu
Tall = Rnedenu*Ruasned*Rguas*Tcg + Rnedenu*Ruasned*Tguas + Rnedenu*Tuasned + Tnedenu;
%In order to determine the xENU and uENU, the zC factor must be determined
zTall = Tall(3);
zPprimeENU = PprimeENU(3);
zC = (zENU-zTall)/(zPprimeENU-zTall);
xENU = zC*PprimeENU(1)-zC*Tall(1)+Tall(1);
yENU = zC*PprimeENU(2)-zC*Tall(2)+Tall(2);
PENU = [xENU; yENU; zENU];
display('The georeferenced pixel position in the ENU frame is given by:  ');
display(PENU);