%% Path Length test for start- and goal- point goes towards each other
%% Big Map
clear
load('../build-map_control-Desktop-Debug/Boustrophedon_length_test_rand_bigMap.txt');
load('../build-map_control-Desktop-Debug/voronoi_length_test_rand_bigMap.txt');
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
title('Big Map: Length in pixels function as Start- and End- points data number, Sample = 6027'); % Random number generate = 10000
a = plot(0:size(voronoi_length_test_rand_bigMap)-1, Boustrophedon_length_test_rand_bigMap)
plot(0:size(voronoi_length_test_rand_bigMap)-1, voronoi_length_test_rand_bigMap)
plot(0:size(avg)-1, avg)
legend('Boustrophedon roadmap', 'Voronoi roadmap', 'Boustrophedon roadmap with moving average (50)');
xlabel('Start- and End- points data number');
ylabel('Total length in pixels');
hold off
a.Color(4) = 0.13;

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
load('../build-map_control-Desktop-Debug/Boustrophedon_length_test_rand_bigMap2.txt');
load('../build-map_control-Desktop-Debug/voronoi_length_test_rand_bigMap2.txt');

avg = movmean(Boustrophedon_length_test_rand_bigMap2, 50); % Moving average over 50 points

figure('Name','Random start- and- end points Big_Map_2');
hold on
title('Big Map 2: Length in pixels function as Start- and End- points data number, Sample = 5282'); % Random number generate = 10000
a = plot(0:1:size(voronoi_length_test_rand_bigMap2)-1, Boustrophedon_length_test_rand_bigMap2)
plot(0:1:size(voronoi_length_test_rand_bigMap2)-1, voronoi_length_test_rand_bigMap2)
plot(0:1:size(avg)-1, avg)
legend('Boustrophedon roadmap', 'Voronoi roadmap', 'Boustrophedon roadmap with moving average (50)');
hold off
xlabel('Start- and End- points data number');
ylabel('Total length in pixels');
a.Color(4) = 0.13;

Boust = sum(Boustrophedon_length_test_rand_bigMap2-voronoi_length_test_rand_bigMap2 < 0);
Voro = sum(Boustrophedon_length_test_rand_bigMap2-voronoi_length_test_rand_bigMap2 > 0);
Equal = sum(Boustrophedon_length_test_rand_bigMap2-voronoi_length_test_rand_bigMap2 == 0);

BigMap2_EQPercent = Equal/size(voronoi_length_test_rand_bigMap2,1)*100
BigMap2_BoustPercent = Boust/size(voronoi_length_test_rand_bigMap2,1)*100
BigMap2_VoroPercent = Voro/size(voronoi_length_test_rand_bigMap2,1)*100

%% Big Map3
clear
load('../build-map_control-Desktop-Debug/Boustrophedon_length_test_rand_bigMap3.txt');
load('../build-map_control-Desktop-Debug/voronoi_length_test_rand_bigMap3.txt');

avg = movmean(Boustrophedon_length_test_rand_bigMap3, 50); % Moving average over 50 points

figure('Name','Random start- and- end points Big_Map_3');
hold on
title('Big Map 3: Length in pixels function as Start- and End- points data number, Sample = 5220'); % Random number generate = 10000
a = plot(0:1:size(voronoi_length_test_rand_bigMap3)-1, Boustrophedon_length_test_rand_bigMap3)
plot(0:1:size(voronoi_length_test_rand_bigMap3)-1, voronoi_length_test_rand_bigMap3)
plot(0:1:size(avg)-1, avg)
legend('Boustrophedon roadmap', 'Voronoi roadmap', 'Boustrophedon roadmap with moving average (50)');
hold off
xlabel('Start- and End- points data number');
ylabel('Total length in pixels');
a.Color(4) = 0.13;

Boust = sum(Boustrophedon_length_test_rand_bigMap3-voronoi_length_test_rand_bigMap3 < 0);
Voro = sum(Boustrophedon_length_test_rand_bigMap3-voronoi_length_test_rand_bigMap3 > 0);
Equal = sum(Boustrophedon_length_test_rand_bigMap3-voronoi_length_test_rand_bigMap3 == 0);

BigMap3_EQPercent = Equal/size(voronoi_length_test_rand_bigMap3,1)*100
BigMap3_BoustPercent = Boust/size(voronoi_length_test_rand_bigMap3,1)*100
BigMap3_VoroPercent = Voro/size(voronoi_length_test_rand_bigMap3,1)*100

%% Small Map
% clear
% load('../build-map_control-Desktop-Debug/Boustrophedon_length_test_rand_smallMap.txt');
% load('../build-map_control-Desktop-Debug/voronoi_length_test_rand_smallMap.txt');
% 
% avg = movmean(Boustrophedon_length_test_rand_smallMap, 50); % Moving average over 50 points
% 
% figure('Name','Random start- and- end points smallMap');
% hold on
% title('Small Map: Length in pixels function as Start- and End- points data number, Sample = 299'); % Random number generate = 500 because of smaller map
% a = plot(0:1:size(voronoi_length_test_rand_smallMap)-1, Boustrophedon_length_test_rand_smallMap)
% plot(0:1:size(voronoi_length_test_rand_smallMap)-1, voronoi_length_test_rand_smallMap)
% plot(0:1:size(avg)-1, avg)
% legend('Boustrophedon roadmap', 'Voronoi roadmap', 'Boustrophedon roadmap with moving average (50)');
% hold off
% xlabel('Start- and End- points data number');
% ylabel('Total length in pixels');
% a.Color(4) = 0.13;
% 
% Boust = sum(Boustrophedon_length_test_rand_smallMap-voronoi_length_test_rand_smallMap < 0);
% Voro = sum(Boustrophedon_length_test_rand_smallMap-voronoi_length_test_rand_smallMap > 0);
% Equal = sum(Boustrophedon_length_test_rand_smallMap-voronoi_length_test_rand_smallMap == 0);
% 
% SmallMap_EQPercent = Equal/size(voronoi_length_test_rand_smallMap,1)*100
% SmallMap_BoustPercent = Boust/size(voronoi_length_test_rand_smallMap,1)*100
% SmallMap_VoroPercent = Voro/size(voronoi_length_test_rand_smallMap,1)*100