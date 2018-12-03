%% Path Length test for start- and goal- point goes towards each other
load('../build-map_control-Desktop-Debug/Boustrophedon_length_test.txt');
load('../build-map_control-Desktop-Debug/voronoi_length_test.txt');
load('../build-map_control-Desktop-Debug/Boustrophedon_length_test_rand.txt');
load('../build-map_control-Desktop-Debug/voronoi_length_test_rand.txt');
figure('Name','Length towards each other');
hold on
plot(1:size(voronoi_length_test), Boustrophedon_length_test)
plot(1:size(voronoi_length_test), voronoi_length_test)
legend('Boustrophedon', 'voronoi');
hold off

% Difference between length towards each other
% hold on
% figure(2)
% plot(1:size(voronoi_length_test), Boustrophedon_length_test-voronoi_length_test)
% hold off

figure('Name','Random start- and- end points');
hold on
plot(Boustrophedon_length_test_rand)
plot(1:size(voronoi_length_test_rand), voronoi_length_test_rand)
legend('Boustrophedon', 'voronoi');
hold off