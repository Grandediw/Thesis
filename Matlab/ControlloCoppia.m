% close all
% clear all
% clc
% 
% a = arduino("/dev/cu.usbmodem1101","Uno","Libraries",);
% 
% 
% writePWMVoltage(a,"D9",0)
% writeDigitalPin(a,"D2",0);
% writeDigitalPin(a,"D4",1);
% for i=1:1:150
%     Current(i) = readVoltage(a,"A0");
% end

%% Impostazioni iniziali

clear all;
clc;

% Definisci costanti
PWM_PIN = 'D9';
IN1_PIN = 'D2';
IN2_PIN = 'D4';
CURRENT_PIN = 'A0';
CURRENT_MAX = 2; % corrente massima sostenibile dal motore
PWM_MAX = 5; % massimo valore di PWM per il controllo del motore

% Definisci parametri del regolatore PI
Kp = 5;
Ki = 1;

% Definisci il setpoint di corrente desiderato
setpoint = 0.16;

% Definisci il periodo di campionamento
Ts = 0.01;

% Inizializza la connessione con Arduino
a = arduino("/dev/cu.usbmodem1301","Uno");

% Imposta i pin di uscita
configurePin(a, PWM_PIN, 'PWM');
configurePin(a, IN1_PIN, 'DigitalOutput');
configurePin(a, IN2_PIN, 'DigitalOutput');

% Imposta il pin di ingresso per il sensore di corrente
configurePin(a, CURRENT_PIN, 'AnalogInput');

%% Controllo del motore

% Inizializza le variabili
error_prev = 0;
integral = 0;
count = 0;
time = 0;
current_data = [];

while time < 2
    
    % Leggi il valore di corrente dal sensore
    current = readVoltage(a, CURRENT_PIN);
    current = (current - 2.5) / 0.185; % converte la tensione letta in corrente (in Ampere)
    
    
    % Calcola l'errore e l'integrale dell'errore
    error = setpoint - current;
    integral = integral + error * Ts;
    
    % Calcola l'azione di controllo
    control_action = Kp*error + Ki*integral;
    control_action = min(max(control_action, -PWM_MAX), PWM_MAX); % Limita l'azione di controllo al range del PWM
    
    % Invia l'azione di controllo al motore
    if control_action >= 0
        writeDigitalPin(a, IN1_PIN, 1);
        writeDigitalPin(a, IN2_PIN, 0);
        writePWMVoltage(a, PWM_PIN, control_action);
    else
        writeDigitalPin(a, IN1_PIN, 0);
        writeDigitalPin(a, IN2_PIN, 1);
        writePWMVoltage(a, PWM_PIN, -control_action);
    end
    
    % Aggiungi i dati di corrente alla matrice di dati
    current_data = [current_data; current];
    
    % Aggiorna le variabili per la prossima iterazione
    error_prev = error;
    count = count + 1;
    time = count*Ts;
    
end

% Stoppa il motore
writeDigitalPin(a, IN1_PIN, 0);
writeDigitalPin(a, IN2_PIN, 0);
writePWMVoltage(a, PWM_PIN, 0);

%%current_data = smoothdata(current_data,"movmean",6);

% Plotta i dati di corrente
plot(0:Ts:(time-Ts), current_data);
xlabel('Tempo (s)');
ylabel('Corrente (A)');
hold on
plot(0:Ts:(time-Ts), setpoint);
title('Prova del controllo della corrente del motore');


% Chiudi la connessione con Arduino
clear a;
%%
mean(current_data)
