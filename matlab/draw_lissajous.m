function draw_lissajous(fname)
% draws a lissajous pattern from parameter file

fp = fopen(fname);
P=fread(fp,10,'double=>real*4');
fclose(fp);

disp(P);

X=lissajous(P(1),P(2), P(3), P(4), P(5), P(6), P(7), P(8),P(9), 20,1);

