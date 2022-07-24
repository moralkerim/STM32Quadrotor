function [xned,yned] = gps2ned(lla,lla0)
%UNTİTLED Summary of this function goes here
%   Detailed explanation goes here
a  = 6378000;
b  = 6357000;
r = 6371000;

lat = deg2rad(lla(1));
lon = deg2rad(lla(2));
h = lla(3);

lat0 = deg2rad(lla0(1));
lon0 = deg2rad(lla0(2));
h0 = lla0(3);

e = 1-(b^2/a^2)
N = a/(sqrt(1-(e^2)*sin(lat)^2))

xecef = (N+h)*cos(lat)*cos(lon);
yecef = (N+h)*cos(lat)*sin(lon);
zecef = ((1-e^2)*N+h)*sin(lat);


x0ecef = (N+h0)*cos(lat0)*cos(lon0);
y0ecef = (N+h0)*cos(lat0)*sin(lon0);
z0ecef = ((1-e^2)*N+h0)*sin(lat0);

dxecef = xecef-x0ecef;
dyecef = yecef-y0ecef;
dzecef = zecef-z0ecef;

yned = -sin(lon0)*dxecef + cos(lon0)*dyecef;
xned = -sin(lat0)*cos(lon0)*dxecef - sin(lat0)*sin(lon0)*dyecef + cos(lat0)*dzecef;
end

