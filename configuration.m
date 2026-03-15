clc; clear; close all;

% ================= 1. KHAI BÁO THÔNG SỐ (PHẢI KHỚP SIMULINK) =================
l1 = 1.0;   % Link 1
l2 = 0.8;   % Link 2
L_obj = 0.2; % Chiều dài vật

% --- QUAN TRỌNG: Khoảng cách giữa 2 chân đế ---
% Vì gốc đặt tại chân Robot 1 (0,0). Robot 2 nằm trên trục X.
Base_Dist = 2.0; % 

% --- VỊ TRÍ VẬT BAN ĐẦU (Start_Val trong Simulink) ---
% Ví dụ vật nằm giữa 2 robot, cao 1m
xc_start = 1;    % Tọa độ X của tâm vật (World Frame) 1
yc_start = 0.8;    % Tọa độ Y của tâm vật (World Frame) 0.8
th_start = 0;      % Góc nghiêng vật (rad) 0
% [1;  1.45; 0.8];
% [1.2;  0.55; 0];
% ================= 2. TÍNH TOÁN ĐIỂM KẸP (Tip Position) =================
% Tính trong hệ tọa độ World
% Tip 1: Bên trái vật
xt1 = xc_start - (L_obj/2) * cos(th_start);
yt1 = yc_start - (L_obj/2) * sin(th_start);

% Tip 2: Bên phải vật
xt2 = xc_start + (L_obj/2) * cos(th_start);
yt2 = yc_start + (L_obj/2) * sin(th_start);

% ================= 3. GIẢI INVERSE KINEMATICS =================

% --- ROBOT 1 (Gốc tại 0,0) ---
% Mục tiêu: Với tới (xt1, yt1)
[q1_init, q2_init, err1] = solve_IK_2link(xt1, yt1, l1, l2, 'ElbowUp');

% --- ROBOT 2 (Gốc tại Base_Dist, 0) ---
% Mục tiêu: Với tới (xt2, yt2)
% CHUYỂN TRỤC: Tọa độ của điểm kẹp 2 so với chân đế Robot 2
x_local2 = xt2 - Base_Dist; 
y_local2 = yt2;
[q3_init, q4_init, err2] = solve_IK_2link(x_local2, y_local2, l1, l2, 'ElbowUp');

% ================= 4. XUẤT KẾT QUẢ =================
if err1 || err2
    disp('❌ LỖI: Điểm đặt nằm ngoài tầm với của Robot!');
    disp('Hãy chỉnh lại xc_start, yc_start hoặc Base_Dist.');
else
    fprintf('=========== KẾT QUẢ INITIAL CONDITIONS ===========\n');
    fprintf('Copy các giá trị này vào khối Integrator trong Simulink:\n');
    fprintf('q1_init = %.4f\n', q1_init);
    fprintf('q2_init = %.4f\n', q2_init);
    fprintf('q3_init = %.4f\n', q3_init);
    fprintf('q4_init = %.4f\n', q4_init);
    fprintf('==================================================\n');
    
    % --- VẼ KIỂM TRA (Visual Check) ---
    figure(1); hold on; axis equal; grid on;
    % Vẽ Robot 1
    plot([0, l1*cos(q1_init)], [0, l1*sin(q1_init)], 'b-o', 'LineWidth', 2);
    plot([l1*cos(q1_init), xt1], [l1*sin(q1_init), yt1], 'b-o', 'LineWidth', 2);
    
    % Vẽ Robot 2
    % Base 2
    b2x = Base_Dist; b2y = 0;
    % Elbow 2 (tính theo góc cục bộ cộng với tịnh tiến)
    e2x = b2x + l1*cos(q3_init);
    e2y = b2y + l1*sin(q3_init);
    % Tip 2 (kiểm tra lại forward kinematics)
    t2x = e2x + l2*cos(q3_init + q4_init);
    t2y = e2y + l2*sin(q3_init + q4_init);
    
    plot([b2x, e2x], [b2y, e2y], 'r-o', 'LineWidth', 2);
    plot([e2x, t2x], [e2y, t2y], 'r-o', 'LineWidth', 2);
    
    % Vẽ Vật
    plot([xt1, xt2], [yt1, yt2], 'k-', 'LineWidth', 4);
    xlabel('World X'); ylabel('World Y');
    title('Kiểm tra tư thế ban đầu (Xanh: Robot 1, Đỏ: Robot 2)');
end

% ================= HÀM CON GIẢI IK =================
function [q1, q2, error_flag] = solve_IK_2link(x, y, l1, l2, config)
    error_flag = 0;
    % Định lý hàm Cosin
    r_sq = x^2 + y^2;
    cos_q2 = (r_sq - l1^2 - l2^2) / (2*l1*l2);
    
    if abs(cos_q2) > 1
        q1 = 0; q2 = 0;
        error_flag = 1;
        return;
    end
    
    % Tính q2
    if strcmp(config, 'ElbowUp')
        q2 = -acos(cos_q2); % Thường giá trị âm là khuỷu gập lên (tùy hệ quy chiếu)
        % Nếu thấy robot bị gập đầu gối xuống đất, đổi dấu thành +acos
    else
        q2 = acos(cos_q2);
    end
    
    % Tính q1
    k1 = l1 + l2*cos(q2);
    k2 = l2*sin(q2);
    q1 = atan2(y, x) - atan2(k2, k1);
end
