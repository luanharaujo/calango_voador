clear
close all
clc
system('sshpass -p calango scp debian@192.168.0.106:/var/lib/cloud9/calango_voador/beaglebone/bin/data.csv ./');
data = readtable('data.csv');
[row,col] = size(data);
labels = data.Properties.VariableNames;
data = table2array(data);

ind = data(1,2:col);

for i = 2:col
    figure(ind(i-1));
    hold all
    plot(data(2:row,1), data(2:row,i), 'DisplayName', labels{i})
   % get(legend(gca),'String'); 
   % legend(labels{i-1});
   legend show
   xlabel(labels{1})
end