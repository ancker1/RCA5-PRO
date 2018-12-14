%% Path Length test for start- and goal- point goes towards each other
%% Big Map
clear
load('Boustrophedon_length_test_rand_bigMap.txt');
load('voronoi_length_test_rand_bigMap.txt');
% figure('Name','Length towards each other');
% hold on
% plot(1:size(voronoi_length_test), Boustrophedon_length_test)
% plot(1:size(voronoi_length_test), voronoi_length_test)
% legend('Boustrophedon', 'voronoi');
% hold off

% Difference between length towards each other
% hold on
% figure(2)
% plot(1:size(voronoi_length_test), Boustrophedon_length_test-voronoi_length_test)
% hold off
avg = movmean(Boustrophedon_length_test_rand_bigMap, 50); % Moving average over 50 points
figure('Name','Random start- and- end points Big_Map_1');
hold on
title('Length in pixels versus Sample number, Sample = 5991'); % Random number generate = 10000
a = plot(0:size(voronoi_length_test_rand_bigMap)-1, Boustrophedon_length_test_rand_bigMap)
plot(0:size(voronoi_length_test_rand_bigMap)-1, voronoi_length_test_rand_bigMap)
plot(0:size(avg)-1, avg)
legend('Boustrophedon road map', 'Voronoi road map', 'Boustrophedon road map with moving average (50)');
xlabel('Sample number');
ylabel('Total length in pixels');
set(gca,'FontSize',20)
hold off
a.Color(4) = 0.13;

sumBoust = sum(Boustrophedon_length_test_rand_bigMap);
sumVoro = sum(voronoi_length_test_rand_bigMap);

Boust = sum(Boustrophedon_length_test_rand_bigMap-voronoi_length_test_rand_bigMap < 0);
Voro = sum(Boustrophedon_length_test_rand_bigMap-voronoi_length_test_rand_bigMap > 0);
Equal = sum(Boustrophedon_length_test_rand_bigMap-voronoi_length_test_rand_bigMap == 0);

BigMap_EQPercent = Equal/size(voronoi_length_test_rand_bigMap,1)*100
BigMap_BoustPercent = Boust/size(voronoi_length_test_rand_bigMap,1)*100
BigMap_VoroPercent = Voro/size(voronoi_length_test_rand_bigMap,1)*100

[vMax,iMax] = max(Boustrophedon_length_test_rand_bigMap-voronoi_length_test_rand_bigMap) % -1 because of zero index
[vMin,iMin] = min(Boustrophedon_length_test_rand_bigMap-voronoi_length_test_rand_bigMap) % -1 because of zero index
%% Big Map2
clear
load('Boustrophedon_length_test_rand_bigMap2.txt');
load('voronoi_length_test_rand_bigMap2.txt');

avg = movmean(Boustrophedon_length_test_rand_bigMap2, 50); % Moving average over 50 points

figure('Name','Random start- and- end points Big_Map_2');
hold on
title('Length in pixels versus Sample number, Sample = 5280'); % Random number generate = 10000
a = plot(0:1:size(voronoi_length_test_rand_bigMap2)-1, Boustrophedon_length_test_rand_bigMap2)
plot(0:1:size(voronoi_length_test_rand_bigMap2)-1, voronoi_length_test_rand_bigMap2)
plot(0:1:size(avg)-1, avg)
legend('Boustrophedon roadmap', 'Voronoi roadmap', 'Boustrophedon roadmap with moving average (50)');
hold off
xlabel('Sample number');
ylabel('Total length in pixels');
set(gca,'FontSize',20)
a.Color(4) = 0.13;

sumBoust = sum(Boustrophedon_length_test_rand_bigMap2);
sumVoro = sum(voronoi_length_test_rand_bigMap2);

Boust = sum(Boustrophedon_length_test_rand_bigMap2-voronoi_length_test_rand_bigMap2 < 0);
Voro = sum(Boustrophedon_length_test_rand_bigMap2-voronoi_length_test_rand_bigMap2 > 0);
Equal = sum(Boustrophedon_length_test_rand_bigMap2-voronoi_length_test_rand_bigMap2 == 0);

BigMap2_EQPercent = Equal/size(voronoi_length_test_rand_bigMap2,1)*100
BigMap2_BoustPercent = Boust/size(voronoi_length_test_rand_bigMap2,1)*100
BigMap2_VoroPercent = Voro/size(voronoi_length_test_rand_bigMap2,1)*100

%% Big Map3
clear
load('Boustrophedon_length_test_rand_bigMap3.txt');
load('voronoi_length_test_rand_bigMap3.txt');

avg = movmean(Boustrophedon_length_test_rand_bigMap3, 50); % Moving average over 50 points

figure('Name','Random start- and- end points Big_Map_3');
hold on
title('Length in pixels versus Sample number, Sample = 5220'); % Random number generate = 10000
a = plot(0:1:size(voronoi_length_test_rand_bigMap3)-1, Boustrophedon_length_test_rand_bigMap3)
plot(0:1:size(voronoi_length_test_rand_bigMap3)-1, voronoi_length_test_rand_bigMap3)
plot(0:1:size(avg)-1, avg)
legend('Boustrophedon roadmap', 'Voronoi roadmap', 'Boustrophedon roadmap with moving average (50)');
hold off
xlabel('Sample number');
ylabel('Total length in pixels');
set(gca,'FontSize',20)
a.Color(4) = 0.13;


sumBoust = sum(Boustrophedon_length_test_rand_bigMap3);
sumVoro = sum(voronoi_length_test_rand_bigMap3);

Boust = sum(Boustrophedon_length_test_rand_bigMap3-voronoi_length_test_rand_bigMap3 < 0);
Voro = sum(Boustrophedon_length_test_rand_bigMap3-voronoi_length_test_rand_bigMap3 > 0);
Equal = sum(Boustrophedon_length_test_rand_bigMap3-voronoi_length_test_rand_bigMap3 == 0);

BigMap3_EQPercent = Equal/size(voronoi_length_test_rand_bigMap3,1)*100
BigMap3_BoustPercent = Boust/size(voronoi_length_test_rand_bigMap3,1)*100
BigMap3_VoroPercent = Voro/size(voronoi_length_test_rand_bigMap3,1)*100

