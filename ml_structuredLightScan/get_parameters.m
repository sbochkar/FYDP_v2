load('Cam_Proj_calib');
%same variale names (that also represent same data)
%nx_proj
%ny_proj
%nx_cam
%ny_cam
%fc_proj
%fc_cam
%cc_proj
%cc_cam
%kc_proj
%kc_cam
%alpha_c_proj
%alpha_c_cam

Rc_1_proj = R_proj;
Tc_1_proj = T_proj;

%Rc_1_cam = [ 0.049112 	 0.919185 	 0.390752; ...
%             0.990403 	 0.005785 	 -0.138088; ...
%            -0.129189 	 0.393784 	 -0.910079 ];
Rc_1_cam = [ -0.128057 	 0.641976 	 -0.755955; ...
              0.991557 	 0.098556 	 -0.084271; ...
              0.020404 	 -0.760364 	 -0.649177 ];
%Tc_1_cam = [ -78.893576 	 -284.966378 	 967.047430 ]';

Tc_1_cam = [ 93.553238 	 -87.370564 	 518.283447 ]';

dX_cam = 17;
dY_cam = 17;
dX_proj = 30;
dY_proj = 30;
addpath('../utilities');




c = 1:nx_proj;
r = 1:ny_proj;
[C,R] = meshgrid(c,r);
np  = pixel2ray([C(:) R(:)]',fc_proj,cc_proj,kc_proj,alpha_c_proj);
np = Rc_1_proj'*(np - Tc_1_proj*ones(1,size(np,2)));
Np = zeros([ny_proj nx_proj 3]);
Np(:,:,1) = reshape(np(1,:),ny_proj,nx_proj);
Np(:,:,2) = reshape(np(2,:),ny_proj,nx_proj);
Np(:,:,3) = reshape(np(3,:),ny_proj,nx_proj);
P = -Rc_1_proj'*Tc_1_proj;

% Estimate plane equations describing every projector column.
% Note: Resulting coefficient vector is in camera coordinates.
wPlaneCol = zeros(nx_proj,4);
for i = 1:nx_proj
   wPlaneCol(i,:) = ...
      fitPlane([P(1); Np(:,i,1)],[P(2); Np(:,i,2)],[P(3); Np(:,i,3)]);
   %figure(4); hold on;
   %plot3(Np(:,i,1),Np(:,i,3),-Np(:,i,2),'r-');
   %drawnow;
end

% Estimate plane equations describing every projector row.
% Note: Resulting coefficient vector is in camera coordinates.
wPlaneRow = zeros(ny_proj,4);
for i = 1:ny_proj
   wPlaneRow(i,:) = ...
      fitPlane([P(1) Np(i,:,1)],[P(2) Np(i,:,2)],[P(3) Np(i,:,3)]);
   %figure(4); hold on;
   %plot3(Np(i,:,1),Np(i,:,3),-Np(i,:,2),'g-');
   %drawnow;
end

% Pre-compute optical rays for each camera pixel.
   c = 1:nx_cam;
   r = 1:ny_cam;
   [C,R] = meshgrid(c,r);
   Nc = Rc_1_cam*Rc_1_cam'*pixel2ray([C(:) R(:)]'-1,fc_cam,cc_cam,kc_cam,alpha_c_cam);
   Oc = Rc_1_cam*Rc_1_cam'*(-Tc_1_cam) + Tc_1_cam;