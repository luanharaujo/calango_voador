clear
clc
close all
%criando o objeto tcpip com os parametros corretos para a comunicação com o rasp
t = tcpip('192.168.8.1', 9001, 'NetworkRole', 'client'); 
%abrindo o  canal de comunicação
fopen(t);

tempo = zeros(1,150);
ref = zeros(1,150);
theta = zeros(1,150); 

figure(1), hold on
i=0;
while true
    i=i+1;
    %recebendo os dados 
    data = fread(t, 30);
    tdata = char(data');
    %i=i+1
    %convertendo de char para floats
    datanum = regexp(tdata,'[+-]?\d+\.?\d*', 'match');
    

    tempo = [tempo(2:end),str2num(datanum{1})];
    ref = [ref(2:end),str2num(datanum{2})];
    theta = [theta(2:end),str2num(datanum{3})];



    % tempo(1500) = str2num(datanum{1});
    % ref(1500) = str2num(datanum{2});
    % theta(1500) = str2num(datanum{3});
    
    if i==2
        plot(tempo, ref), hold on;
        ylim([-40 130]);
        xlim([tempo(1) tempo(end)]);
        plot(tempo, theta);
        legend('Referência', 'Yaw');
        xlabel('Tempo (s)');
        ylabel('Ângulo (graus)');
        title('Dados em "Tempo Real"');
        drawnow;
        hold off;
        i=0;
    end
    % %
    
    %pause(0.05)
end
