function v = LeakyReLU(w,alpha)
% alpha =0.45;
v = w;
v(find(w<0)) = v(find(w<0))*alpha;
end

