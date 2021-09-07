numVars = 6;
varNames = {'field.data0.pose.position.x', 'field.data0.pose.position.y', 'field.data0.pose.position.z', 'gt.x', 'gt.y', 'gt.z'};
varTypes = {'double', 'double', 'double', 'double', 'double', 'double'};
dataStart = 'E2';
count = 10393;

opts = spreadsheetImportOptions('NumVariables',numVars,...
                                'VariableNames',varNames,...
                                'VariableTypes',varTypes,...
                                'DataRange', dataStart);
                            
plot_data = readtable('C:\Users\Junyoung\Desktop\matlab\trial5.xls', opts);
plot_data_arr = table2array(plot_data(:,:));

X = zeros(count,1);
Y = zeros(count, 1);
Z = zeros(count, 1);
gt_x = zeros(count,1);
gt_y = zeros(count,1);
gt_z = zeros(count,1);

for i = 1:count
   X(i) = plot_data_arr(i,1);
   Y(i) = plot_data_arr(i,2);
   Z(i) = plot_data_arr(i,3);
   gt_x(i) = plot_data_arr(i,4);
   gt_y(i) = plot_data_arr(i,5);
   gt_z(i) = plot_data_arr(i,6);
end

X_anchor = [0,0,4758,256,4728,5959];
Y_anchor = [0,9919,-338,4232,10301,4043];
Z_anchor = [380,820,2550,1800,2550,1140];

figure

scatter3(X,Y,Z,'MarkerEdgeColor',[0.9290 0.6940 0.1250]);  % PREDICIONT

title('tag positioning test');
xlabel('X(mm)');
ylabel('Y(mm)');
zlabel('Z(mm)');
pbaspect([1 2 1]);

hold on
scatter3(X_anchor, Y_anchor, Z_anchor, 100, '^', 'MarkerFaceColor',[0.8500 0.3250 0.0980]);  % ANCHOR\
scatter3(gt_y,gt_x,gt_z,'MarkerEdgeColor',[0.5 0.5 0.5]);  % PREDICION
% scatter3([2700], [3980], [1100], 100, 'MarkerFaceColor', [0 0 0]);  % GORUND TRUTH
legend({'tag','anchor','ground truth'},'FontSize', 12, 'Location','southwest');
view(3)
hold off
