%% Calibration for Kilian

height_k = 1.87;
depth = 0.65:0.65:6*0.65;
measure_px_k = [480, 480, 410, 320, 270, 225]./height_k;

threshold = 1.5; % the distance after which the subject is fully visible

index_k = depth>threshold;
depth_interest_k = depth(index_k);
px_interest_k = measure_px_k(index_k);

figure(1)
hold on;
plot(measure_px_k, depth, 'k-');
plot(px_interest_k,depth_interest_k, 'r.', MarkerSize=10)


approx_k = fit(px_interest_k', depth_interest_k', 'poly1');
y_fit_k = feval(approx_k, px_interest_k);
origin_k = approx_k.p2;
plot(px_interest_k, y_fit_k, 'm--')

xlabel("Pixels [px]");
ylabel("Distance [m]")
title("Distance Calibration Kilian", FontSize=15)
legend("All measurements", "After thresholding", "Linear approximation")



%% Calibration for Victoria

height_v = 1.7;
depth = 0.65:0.65:6*0.65;
measure_px_v = [480, 444, 365, 300, 263, 207]./height_v;

threshold = 1.5; % the distance after which the subject is fully visible

index_v = depth>threshold;
depth_interest_v = depth(index_v);
px_interest_v = measure_px_v(index_v);

figure(2)
hold on;
plot(measure_px_v, depth, 'k-');
plot(px_interest_v,depth_interest_v, 'r.', MarkerSize=10)


approx_v = fit(px_interest_v', depth_interest_v', 'poly1');
y_fit_v = feval(approx_v, px_interest_v);
origin_v = approx_v.p2;
plot(px_interest_v, y_fit_v, 'm--')

xlabel("Pixels [px]");
ylabel("Distance [m]")
title("Distance Calibration Victoria", FontSize=15)
legend("All measurements", "After thresholding", "Linear approximation")



%% Comparison


figure(3)
hold on;
plot(px_interest_k, y_fit_k, 'm--')
plot(px_interest_v, y_fit_v, 'g--')


xlabel("Pixels [px]");
ylabel("Distance [m]")
title("Comparison linear approximations", FontSize=15)
legend("Kilian", "Victoria")


%% Relation with height

figure(4);
hold on;

heights = [height_k,height_v];
origins = [origin_k, origin_v];

plot(heights, origins)

xlabel("Pixels [px]");
ylabel("Distance [m]")
title("Comparison linear approximations", FontSize=15)
