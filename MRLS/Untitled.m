function test_main
% Plots graph and sets up a custom data tip update function
fig = figure();
X = 0:60;
t = (X)*0.02;
Y = sin(-16*t);
plot(X,Y)
dcm_obj = datacursormode(fig);
set(dcm_obj,'UpdateFcn',{@myupdatefcn,t})
function txt = myupdatefcn(~,event_obj,t)
% Customizes text of data tips
pos = get(event_obj,'Position');
I = get(event_obj, 'DataIndex');
txt = {['X: ',num2str(pos(1))],...
       ['Y: ',num2str(pos(2))],...
       ['I: ',num2str(I)],...
       ['T: ',num2str(t(I))]};