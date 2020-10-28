function gf=BFtoGF( qimu_bf_b_bf, q_bf_b)
cc=complexConjugateQuaternion(q_bf_b);
gf=multiplicationQuaternions(qimu_bf_b_bf,cc);

gf = gf/norm(gf);
end