# Unscented-Kalman-Filter-Rust
UKF written in Rust based on the C++ UKF from the Udacity SD Car Nanodegree. The C++ project code this project is based on can be found at https://github.com/hortovanyi/CarND-Unscented-Kalman-Filter-Project.

To build and run this project use
```
cargo run -- --input ./data/sample-laser-radar-measurement-data-1.txt --output output.txt
```
in the project root directory after cloning it.

The above command when run should produce a log with the following output
```
2022-10-18T23:52:31.856127Z  INFO ukf: app_setup
2022-10-18T23:52:31.856195Z  WARN ukf: output_path: output.txt will be overwritten
2022-10-18T23:52:31.856286Z  INFO ukf: processing_started
2022-10-18T23:52:31.856330Z  INFO ukf: loading measurement data ....
2022-10-18T23:52:31.863632Z  INFO ukf: processing measurement data ....
2022-10-18T23:52:31.863670Z  INFO ukf::ukf: init state radar x:[[8.462918745489562, 0.24346236596519058, -3.04035, 0.0287602, 0.0]]
2022-10-18T23:52:31.863726Z  INFO ukf::ukf: init x:
  ┌                     ┐
  │   8.462918745489562 │
  │ 0.24346236596519058 │
  │            -3.04035 │
  │           0.0287602 │
  │                   0 │
  └                     ┘


2022-10-18T23:52:31.863752Z  INFO ukf::ukf: init P:
  ┌           ┐
  │ 1 0 0 0 0 │
  │ 0 1 0 0 0 │
  │ 0 0 1 0 0 │
  │ 0 0 0 1 0 │
  │ 0 0 0 0 1 │
  └           ┘


2022-10-18T23:52:31.863789Z  INFO ukf::ukf: init lidar_sensor:Some(LidarSensor { n_z: 2, std_laspx: 0.15, std_laspy: 0.15 })
2022-10-18T23:52:31.863800Z  INFO ukf::ukf: init radar_sensor:Some(RadarSensor { n_z: 3, std_radr: 0.3, std_radphi: 0.03, std_radrd: 0.3 })
2022-10-18T23:52:32.170115Z  INFO ukf: Accruacy - RMSE: [0.077262500372669, 0.0817972181211013, 0.5892783492841626, 0.5742886905052651]
2022-10-18T23:52:32.170256Z  INFO ukf: processing_finished
```

Note the `Accruacry - RMSE: [0.07726250037271036, 0.08179721812107782, 0.5892783492841654, 0.5742886905052718]`

This project was an iteration on my iniital rust code for a UKF and I simplified the coding to use less Rust synatical sugar.

The code is more concise and formulas are self contained. It has not been abstracted into the types, such as I did initially. That was my first rust project and I was learning. Subsequent attempts to utilise that code for other projects took too long to adapt it. I also found that there was a lot of template code, that whilst nice from a Rust & corretness perspective didnt add to the understandability of what was happening. The additional code was confusing.

Out of the [c++](https://github.com/hortovanyi/CarND-Unscented-Kalman-Filter-Project), [initial rust](https://github.com/hortovanyi/Unscented-Kalman-Filter-Rust), [scala](https://github.com/hortovanyi/Unscented-Kalman-Filter-Spark-Scala) and this version, the most elegant version was the scala code This seemed to be because of interheritance and the use of traits in Scala itself (code was written in Scala 2)

At some point I may try again to iterate on this concise version if I can see merit in doing so and if it adds understandability to the code.