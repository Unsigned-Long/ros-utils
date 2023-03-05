# GS camera
rosrun kalibr kalibr_calibrate_cameras --bag cam_calib_2022-11-08-18-49-53.bag --topics /camera/image --models pinhole-radtan --target ../config/april_6x6.yaml

# RS camera
rosrun kalibr kalibr_calibrate_rs_cameras --bag cam_rs_calib_2023-03-05-19-42-49.bag --topic /camera/image --model pinhole-radtan-rs --target ../config/april_6x6.yaml --frame-rate 10 --inverse-feature-variance 1
