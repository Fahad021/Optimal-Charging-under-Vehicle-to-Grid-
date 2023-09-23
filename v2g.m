clc
clear 
%%
load('As3_data')
load('important_time_2.mat')
home_rp = 1.7;
work_rp = 7.7;
cap = 60*60/5; %kW5min
agg_ld = load_micro;
n = 288;
soc0 = 0.5;
k=1;
lamada=0.01;
Q = zeros(288,100);
for i=1:100
    i
    p_driving = zeros(288,1);
    p_driving(home_lt(i)+1:home_lt(i)+9)= -8;
    p_driving(home_at(i)-9:home_at(i)-1)= -8;
    cvx_begin quiet
    variable q(n) 
         minimize(max(q+agg_ld - p_driving))
        subject to
            q(1:home_lt(i))<=home_rp;
            q(1:home_lt(i))>=-home_rp;  
            q(home_lt(i)+1:home_lt(i)+9)== -8; % driving
            q(home_lt(i)+10:home_at(i)-10)<=work_rp;
            q(home_lt(i)+10:home_at(i)-10)>=-work_rp;  
            q(home_at(i)-9:home_at(i)-1) == -8; % driving
            q(home_at(i):288)<=home_rp;
            q(home_at(i):288)>=-home_rp;  
            soc = soc0 + cumsum(q)/cap;
            soc>=0;
            soc<=1; 
            soc(home_lt(i))>=0.5;
            soc(home_at(i)-9)>=0.5;
    cvx_end    
    
    q_act = q;
    q_act(home_lt(i)+1:home_lt(i)+9)= 0; % driving
    q_act(home_at(i)-9:home_at(i)-1) = 0; % driving
    agg_ld = agg_ld + q_act;
    
    Q(:,k) = q_act; % save the individual EV load profile    
    k = k+1;
end
%%
plot(load_micro/1000,'b')
hold on
plot(agg_ld/1000,'g')

timelabel={'0','4','8','12','16','20','24'};
fs =10;
legend ('Load Profile without EV Penetration','Load Profile with EV Penetration','location','SW')
xlabel('Time','fontsize',fs,'FontName','Times New Roman');
ylabel('Power (MW)','fontsize',fs,'FontName','Times New Roman');
set(gca,'xtick',0:48:288,'xticklabel',timelabel,'fontsize',fs,'FontName','Times New Roman');
axis([0,288,0.5,2.5])