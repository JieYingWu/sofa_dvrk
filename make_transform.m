function out = make_transform(t, r)
    M = zeros(4);
    M(4,4) = 1; 
    M(1:3, 4) = t;
    M(1:3,1:3) = eul2rotm(r);
    out = M;
end