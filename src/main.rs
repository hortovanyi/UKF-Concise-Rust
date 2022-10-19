mod ukf;
mod sensor;

use std::io::{BufRead, BufReader};

use std::{process, usize};

use std::fs::File;
use std::path::Path;
use std::vec::Vec;

pub use sensor::measurement::*;

use clap::{Parser, command};

#[macro_use]
extern crate serde_derive;

use tracing::{debug, error, info, trace, warn, Level};
use tracing_subscriber;

use crate::ukf::UnscentedKalmanFilter;

#[derive(Serialize)]
struct Output {
    px_est: f64,
    py_est: f64,
    vel_abs_est: f64,
    yaw_angle_est: f64,
    yaw_rate_est: f64,
    px_meas: f64,
    py_meas: f64,
    px_gt: f64,
    py_gt: f64,
    vx_gt: f64,
    vy_gt: f64,
    nis_lidar: f64,
    nis_radar: f64,
}

fn run_ukf(input_file: &File, output_file: &File) -> Result<(), String> {
    trace!("run_ukf start");

    let mut measurements: Vec<MeasurementPackage> = Vec::new();
    let mut ground_truths: Vec<GroudTruthPackage> = Vec::new();
    let mut estimations: Vec<EstimationPackage> = Vec::new();

    let reader = BufReader::new(input_file);
    let mut wtr = csv::WriterBuilder::new()
        .delimiter(b'\t')
        .from_writer(output_file);

    info!("loading measurement data ....");
    reader.lines().into_iter().for_each(|line| {
        let l = line.unwrap();
        let mp = MeasurementPackage::from_csv_string(l.clone());
        let gtp = GroudTruthPackage::from_csv_string(l.clone());
        // trace!("{} {:?} {:?}",l, mp, gtp);
        measurements.push(mp);
        ground_truths.push(gtp);
    });

    trace!("creating lidar sensor");
    let lidar_sensor = LidarSensor::new();

    trace!("creating radar sensor");
    let radar_sensor = RadarSensor::new();

    trace!("creating ukf object");
    let mut ukf: UnscentedKalmanFilter = UnscentedKalmanFilter::new(Some(lidar_sensor), Some(radar_sensor));
    info!("processing measurement data ....");
    for (i, m) in measurements.iter().enumerate() {
        debug!("measurement i: {}", i);

        let (x,P) = ukf.process_measurement(m);
        // trace!("{} x:{:?} ", i, x);
        let xest = EstimationPackage::from_state(&x);
        estimations.push(xest);

        let (px_meas, py_meas) = match m.sensor_type {
            SensorType::Lidar => m.lidar_data.as_ref().unwrap().point(),
            SensorType::Radar => m.radar_data.as_ref().unwrap().point(),
        };

        let gtp = &ground_truths[i];
        let output = Output {
            px_est: x[0],
            py_est: x[1],
            vel_abs_est: x[2],
            yaw_angle_est: x[3],
            yaw_rate_est: x[4],
            px_meas: px_meas,
            py_meas: py_meas,
            px_gt: gtp.x,
            py_gt: gtp.y,
            vx_gt: gtp.vx,
            vy_gt: gtp.vy,
            nis_lidar: ukf.nis_lidar,
            nis_radar: ukf.nis_radar,
        };
        wtr.serialize(output).unwrap();

        // if i > 10 {break};
    }

    info!(
        "Accruacy - RMSE: {:?}",
        ukf::util::calculate_rmse(&estimations, &ground_truths)
    );
    wtr.flush().unwrap();

    trace!("run_ukf finish");
    Ok(())
}

fn run(args: Arguments) -> Result<(), String> {
    let min_log_level = match args.verbose.unwrap_or_default(){
        0 => Level::INFO,
        1 => Level::DEBUG,
        2 | _ => Level::TRACE,
    };

    // install global collector configured based on RUST_LOG env var.
    tracing_subscriber::fmt()
        .with_max_level(min_log_level)
        .init();

    debug!("tracing::Level::{}", min_log_level.as_str());

    info!("app_setup");
    let input_file_name = args.input;
    let input_path = Path::new(&input_file_name);
    if !input_path.is_file() {
        error!(
            "input_path: {} does not exist or isn't a file",
            input_path.display()
        );
        panic!("no input file!");
    }

    let output_file_name = args.output;
    let output_path = Path::new(&output_file_name);
    if output_path.is_file() {
        warn!("output_path: {} will be overwritten", output_path.display());
    }

    debug!("opening input for read: `{}`", input_path.display());
    let input_file = File::open(&input_path).unwrap();

    debug!("creating output: `{}`", output_path.display());
    let output_file = File::create(output_path).unwrap();

    trace!("app_setup_complete");
    // starting processing...
    info!("processing_started");
    run_ukf(&input_file, &output_file)?;
    info!("processing_finished");

    Ok(())
}

#[derive(Parser, Default, Debug, Clone)]
#[clap(
    author = "Nick Hortovanyi",
    version = "0.2.0",
    about = "Concise UKF in Rust based on C++ version"
)]
#[command(author, version, about, long_about= None)]
struct Arguments {
    #[clap(short, long)]
    input: String,
    #[clap(short, long)]
    output: String,
    #[clap(short, long, action = clap::ArgAction::Count)]
    verbose: Option<u8>,
}

fn main() {
    let args = Arguments::parse();
    if let Err(err) = run(args) {
        println!("Application error: {}", err);
        process::exit(1);
    }
}
