function bf=GFtoBF(q_bf_b, qimu_b)
%the sensor-to-body orientation
cc=complexConjugateQuaternion(q_bf_b);
bf=multiplicationQuaternions(cc,qimu_b);

bf=bf/norm(bf);
end