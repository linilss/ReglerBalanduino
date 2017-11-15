function balanduino_plot(logged_data)

time = logged_data(:, 1);
reference = logged_data(:, 2);
actual_output = logged_data(:, 3);
error = logged_data(:, 4);
calculated_control = logged_data(:, 5);
actual_control = logged_data(:, 6);

subplot(3,1,1)
plot(time, reference, time, actual_output)
legend('reference', 'actual output')
title('Reference and actual output')

subplot(3,1,2)
plot(time, error)
legend('error')
title('Control error')



subplot(3,1,3)
plot(time, calculated_control, time, actual_control)
legend('calculated control', 'actual control')
title('Calculated and actual control signals')

