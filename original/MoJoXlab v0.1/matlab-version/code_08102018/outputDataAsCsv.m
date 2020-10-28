%% output data as csv file

M = [Aankle_angle{1}, Aankle_angle{2}, Ahip_angle{1}, Ahip_angle{2}, Aknee_angle{1}, Aknee_angle{2}]
filename = sprintf('jointangles_%s.csv', datestr(now, 'ddmmyyyy_HHMMSS'));
csvwrite(filename,M)