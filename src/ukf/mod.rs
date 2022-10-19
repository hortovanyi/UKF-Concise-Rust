pub mod util;

extern crate nalgebra as na;
use na::OMatrix;
use na::{OVector, U15, U2, U3, U5, U7};

use sensor::measurement::{LidarMeasurement, RadarMeasurement};
use sensor::measurement::{LidarSensor, RadarSensor};

use tracing::{debug, error, info, trace, warn};

use crate::{sensor, HasSensorNoiseCovar, MeasurementPackage, SensorType};

use self::util::negative_normalize;

// U? are for nalegbra

// state dimensions
pub const N_X: usize = 5;
pub type UX = U5;

// augmented state dimensions
pub const N_AUG: usize = 7;
pub type UAUG = U7;

// process noise standard deviation longitudinal acceleration in m/s^2
pub const STD_A: f64 = 0.45;

// process noise standard deviation yaw acceleration in rad/s^2
pub const STD_YAWDD: f64 = 0.55;

// laser measurement noise
pub const STD_LASPX: f64 = 0.15;
pub const STD_LASPY: f64 = 0.15;

// radar measurement noise
pub const STD_RADR: f64 = 0.3;
pub const STD_RADPHI: f64 = 0.03;
pub const STD_RADRD: f64 = 0.3;

// size of lidar and radar measurement state vectors
pub const N_Z_LIDAR: usize = 2;
pub type UZLIDAR = U2;
pub const N_Z_RADAR: usize = 3;
pub type UZRADAR = U3;

// sigma points (2 * N_AUG + 1)
pub type USigmaPoints = U15;

// define the spreading parameter
pub const LAMBDA: f64 = 3.0 - N_AUG as f64;

// state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
pub type StateVector = OVector<f64, UX>;
pub type AugStateVector = OVector<f64, UAUG>;

pub type LidarStateVector = OVector<f64, UZLIDAR>;
pub type RadarStateVector = OVector<f64, UZRADAR>;

// measurement covariance matrices
pub type CovarMatrix = OMatrix<f64, UX, UX>;
pub type AugCovarMatrix = OMatrix<f64, UAUG, UAUG>;
pub type CholeskyMatrix = AugCovarMatrix;
pub type LidarCovarMatrix = OMatrix<f64, UZLIDAR, UZLIDAR>;
pub type LidarNoiseCovarMatrix = LidarCovarMatrix;
pub type RadarCovarMatrix = OMatrix<f64, UZRADAR, UZRADAR>;
pub type RadarNoiseCovarMatrix = RadarCovarMatrix;

// augmented sigma points matrix (n_aug, 2 * n_aug + 1)
pub type AugSigmaPoints = OMatrix<f64, UAUG, USigmaPoints>;

// predicted sigma points matrix (n_x, 2* n_aug + 1)
pub type SigmaPoints = OMatrix<f64, UX, USigmaPoints>;
pub type LidarSigmaPoints = OMatrix<f64, UZLIDAR, USigmaPoints>;
pub type RadarSigmaPoints = OMatrix<f64, UZRADAR, USigmaPoints>;

// sigma point weights
pub type SigmaPointWeights = OVector<f64, USigmaPoints>;

// cross correlation matrix
pub type LidarCrossCorrelationMatrix = OMatrix<f64, UX, UZLIDAR>;
pub type RadarCrossCorrelationMatrix = OMatrix<f64, UX, UZRADAR>;

pub type LidarKalmanGain = OMatrix<f64, UX, UZLIDAR>;
pub type RadarKalmanGain = OMatrix<f64, UX, UZRADAR>;

#[allow(non_snake_case)]
#[derive(Debug)]
pub struct UnscentedKalmanFilter {
    // initially set to false, set to ture in frist call of ProcessMeasurement
    is_initiliased: bool,

    lidar_sensor: Option<LidarSensor>,
    radar_sensor: Option<RadarSensor>,

    // previous timestamp
    prev_timestamp: u64,

    // Covariance Matrix
    P: CovarMatrix,

    // StateVector
    x: StateVector,

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a: f64,

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd: f64,

    // Laser measurement noise standard deviation position1 in m
    std_laspx: f64,

    // Laser measurement noise standard deviation position2 in m
    std_laspy: f64,

    // Radar measurement noise standard deviation radius in m
    std_radr: f64,

    // Radar measurement noise standard deviation angle in rad
    std_radphi: f64,

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd: f64,

    // Weights of sigma points
    weights: SigmaPointWeights,

    // State dimension
    n_x: usize,

    // Augmented state dimension
    n_aug: usize,

    // Normalised Innovation Squared
    pub nis_lidar: f64,
    pub nis_radar: f64,
}

pub trait HasStateVectorColumnSlice {
    fn state_from_col(&self, i: usize) -> StateVector;
}

impl HasStateVectorColumnSlice for SigmaPoints {
    fn state_from_col(&self, col: usize) -> StateVector {
        let mut x = StateVector::zeros();

        for i in 0..x.shape().0 {
            x[i] = self[(i, col)]
        }
        x
    }
}

pub trait NCols {
    fn cols(&self) -> usize;
}
impl NCols for SigmaPoints {
    fn cols(&self) -> usize {
        self.shape().1
    }
}
impl NCols for LidarSigmaPoints {
    fn cols(&self) -> usize {
        self.shape().1
    }
}
impl NCols for RadarSigmaPoints {
    fn cols(&self) -> usize {
        self.shape().1
    }
}

#[allow(non_snake_case)]
impl UnscentedKalmanFilter {
    pub fn new(lidar_sensor: Option<LidarSensor>, radar_sensor: Option<RadarSensor>) -> Self {
        // sigma points weights
        let weight = 0.5 / (N_AUG as f64 + LAMBDA);
        let mut weights = SigmaPointWeights::repeat(weight);
        weights[0] = LAMBDA / (LAMBDA + N_AUG as f64);

        Self {
            is_initiliased: false,
            lidar_sensor,
            radar_sensor,
            prev_timestamp: 0,
            P: CovarMatrix::from_diagonal_element(1.0),
            x: StateVector::zeros(),
            std_a: STD_A,
            std_yawdd: STD_YAWDD,
            std_laspx: STD_LASPX,
            std_laspy: STD_LASPY,
            std_radr: STD_RADR,
            std_radphi: STD_RADPHI,
            std_radrd: STD_RADRD,
            weights: weights,
            n_x: N_X,
            n_aug: N_AUG,
            nis_lidar: 0.0,
            nis_radar: 0.0,
        }
    }

    pub fn process_measurement(&mut self, m: &MeasurementPackage) -> (StateVector, CovarMatrix) {
        // initiliase if it hasnt already occurred
        if self.is_initiliased == false {
            self.init_measurement(m)

        // do predict and update steps
        } else {
            let delta_t: f64 = (m.timestamp - self.prev_timestamp) as f64 / 1000000.0;
            self.prev_timestamp = m.timestamp;

            // prediction step
            let (X_sig_pred, x, P) = self.prediction(delta_t);
            debug!("X_sig_pred: {:?}", X_sig_pred);

            // update step
            let (x, P, nis) = match m.sensor_type {
                SensorType::Lidar => match &self.lidar_sensor {
                    Some(_) => {
                        debug!("lidar_data: {:?}", m.lidar_data.clone());
                        self.lidar_update(m.lidar_data.clone(), X_sig_pred, x, P)
                    }
                    None => {
                        debug!("not using lidar_sensor");
                        (self.x, self.P, 0.0)
                    }
                },
                SensorType::Radar => match &self.radar_sensor {
                    Some(_) => {
                        debug!("radar_data: {:?}", m.radar_data.clone());
                        self.radar_update(m.radar_data.clone(), X_sig_pred, x, P)
                    }
                    None => {
                        debug!("not using radar_sensor");
                        (self.x, self.P, 0.0)
                    }
                },
            };
            self.x = x;
            self.P = P;

            debug!("X_sig_pred: {:?}", X_sig_pred);
            (x, P)
        }
    }

    fn init_measurement(&mut self, m: &MeasurementPackage) -> (StateVector, CovarMatrix) {
        match m.clone().sensor_type {
            SensorType::Lidar => {
                match &m.lidar_data {
                    Some(m) => {
                        self.x[0] = m.px;
                        self.x[1] = m.py;
                    }
                    None => error!("init measurement lidar has None measurement!"),
                }

                info!("init state lidar x:{:?}", self.x);
            }
            SensorType::Radar => {
                match &m.radar_data {
                    Some(m) => {
                        let rho = m.rho; // Range - radial distance from origin
                        let phi = m.theta; // bearing - angle between rho and x
                        let rho_dot = m.rho_dot; // Radial Velocity - change of p(range rate)

                        let px = rho * phi.cos(); // metres
                        let py = rho * phi.sin();
                        let v = rho_dot; // metres/sec
                        let yaw = phi; // radians
                        let yaw_dot = 0.0; // radians/sec

                        self.x[0] = px;
                        self.x[1] = py;
                        self.x[2] = v;
                        self.x[3] = yaw;
                        self.x[4] = yaw_dot;
                    }
                    None => error!("init measurement radar has None measurement!"),
                }

                info!("init state radar x:{:?}", self.x);
            }
        }

        self.prev_timestamp = m.timestamp;
        self.is_initiliased = true;
        info!("init x:{}", self.x);
        info!("init P:{}", self.P);

        info!("init lidar_sensor:{:?}", self.lidar_sensor);
        info!("init radar_sensor:{:?}", self.radar_sensor);
        (self.x, self.P)
    }

    fn prediction(&self, delta_t: f64) -> (SigmaPoints, StateVector, CovarMatrix) {
        //
        // augment sigma points
        //
        // create augmented state vector
        let mut x_aug = AugStateVector::zeros();
        x_aug.copy_from(&self.x.clone().resize(self.n_aug, 1, 0.0));

        // create augmented coveriance matrix
        let mut P_aug = AugCovarMatrix::zeros();
        P_aug.copy_from(&self.P.clone().resize(self.n_aug, self.n_aug, 0.0));
        P_aug[(5, 5)] = self.std_a * self.std_a;
        P_aug[(6, 6)] = self.std_yawdd * self.std_yawdd;

        // square root of P
        let L: CholeskyMatrix = match P_aug.cholesky() {
            Some(x) => x.l(),
            None => CholeskyMatrix::zeros(),
        };

        // define spreading parameter
        let spread = (LAMBDA + self.n_aug as f64).sqrt();

        let mut X_sig_aug = AugSigmaPoints::zeros();

        X_sig_aug.column_mut(0).copy_from(&x_aug);

        for i in 0..self.n_aug {
            let mut x_aug1_col = x_aug.clone();
            let mut x_aug2_col = x_aug.clone();
            for j in 0..self.n_aug {
                x_aug1_col[j] += spread * L.column(i)[j];
                x_aug2_col[j] -= spread * L.column(i)[j];
            }
            X_sig_aug.column_mut(i + 1).copy_from(&x_aug1_col);
            X_sig_aug
                .column_mut(i + 1 + self.n_aug)
                .copy_from(&x_aug2_col);
        }

        //
        // predict sigma points
        //
        let mut X_sig_pred = SigmaPoints::zeros();

        let n_aug = X_sig_aug.shape().0;

        for i in 0..2 * n_aug + 1 {
            // extract values for better readability
            let p_x = X_sig_aug[(0, i)];
            let p_y = X_sig_aug[(1, i)];
            let v = X_sig_aug[(2, i)];
            let yaw = X_sig_aug[(3, i)];
            let yawd = X_sig_aug[(4, i)];
            let nu_a = X_sig_aug[(5, i)];
            let nu_yawdd = X_sig_aug[(6, i)];

            // predicted state values in *_p
            let mut px_p: f64;
            let mut py_p: f64;

            // avoid division by zero
            if yawd.is_normal() {
                px_p = p_x + v / yawd * ((yaw + yawd * delta_t).sin() - yaw.sin());
                py_p = p_y + v / yawd * (yaw.cos() - (yaw + yawd * delta_t).cos());
            } else {
                px_p = p_x + v * delta_t * yaw.cos();
                py_p = p_y + v * delta_t * yaw.sin();
            }

            let mut v_p = v;
            let mut yaw_p = yaw + yawd * delta_t;
            let mut yawd_p = yawd;

            // add noise
            px_p = px_p + 0.5 * nu_a * delta_t * delta_t * yaw.cos();
            py_p = py_p + 0.5 * nu_a * delta_t * delta_t * yaw.sin();
            v_p = v_p + nu_a * delta_t;

            yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
            yawd_p = yawd_p + nu_yawdd * delta_t;

            //write predicted sigma point into right column
            X_sig_pred[(0, i)] = px_p;
            X_sig_pred[(1, i)] = py_p;
            X_sig_pred[(2, i)] = v_p;
            X_sig_pred[(3, i)] = yaw_p;
            X_sig_pred[(4, i)] = yawd_p;
        }

        //
        // predict mean and covariance
        //
        // create the predicted state vector
        let mut x = StateVector::zeros();

        // create the predicted covariance matrix
        let mut P = CovarMatrix::zeros();

        let n_sig_cols = X_sig_pred.cols();

        // predicted state mean
        for i in 0..n_sig_cols {
            x += self.weights[i] * X_sig_pred.state_from_col(i);
        }

        // predicted state covariance matrix
        for i in 1..n_sig_cols {
            // state difference
            let mut x_diff = X_sig_pred.state_from_col(i) - X_sig_pred.state_from_col(0);

            // angle normalization -pi to pi
            x_diff[3] = negative_normalize(x_diff[3]);

            P += self.weights[i] * x_diff * x_diff.transpose();
        }

        (X_sig_pred, x, P)
    }

    fn lidar_update(
        &self,
        m: Option<LidarMeasurement>,
        x_sig_pred: SigmaPoints,
        x: StateVector,
        p: CovarMatrix,
    ) -> (StateVector, CovarMatrix, f64) {
        let z = match m.clone() {
            Some(m) => LidarStateVector::new(m.px, m.py),
            None => LidarStateVector::zeros(),
        };

        debug!("laser z: {:?}", z);

        //
        // predict measurement
        //

        // sigma points laser measurement space
        let mut Z_sig = LidarSigmaPoints::zeros();

        for i in 0..Z_sig.cols() {
            let p_x = x_sig_pred[(0, i)];
            let p_y = x_sig_pred[(1, i)];

            // measurement model
            Z_sig[(0, i)] = p_x;
            Z_sig[(1, i)] = p_y;
        }

        // predict measurement and covariance
        let mut S = LidarCovarMatrix::zeros();

        // measurement covariance matrix S
        for i in 1..Z_sig.cols() {
            let z_diff = Z_sig.column(i) - Z_sig.column(0);
            S += self.weights[i] * z_diff * z_diff.transpose();
        }

        // add measurement noise
        match &self.lidar_sensor {
            Some(sensor) => S += sensor.noise_covar_matrix(),
            None => warn!("no lidar sensor defined - unable to add noise to covariance matrix"),
        };

        // mean predicted measurement
        let mut z_pred = LidarStateVector::zeros();
        for i in 0..Z_sig.cols() {
            z_pred += self.weights[i] * Z_sig.column(i);
        }

        //
        // Update state
        //

        // cross correlation matrix Tc
        let mut Tc = LidarCrossCorrelationMatrix::zeros();
        let n_sig_cols = Z_sig.cols();

        for i in 1..n_sig_cols {
            let z_diff = Z_sig.column(i) - Z_sig.column(0);
            let mut x_diff =
                x_sig_pred.fixed_slice::<N_X, 1>(0, i) - x_sig_pred.fixed_slice::<N_X, 1>(0, 0);
            x_diff[3] = negative_normalize(x_diff[3]);
            Tc += self.weights[i] * x_diff * z_diff.transpose();
        }

        // Kalma gain K
        let K: LidarKalmanGain = Tc
            * match S.try_inverse() {
                Some(s_inverse) => s_inverse,
                None => S,
            };

        // residual
        let z_diff = z - z_pred;

        // update state mean and covariance matrix
        let x = x + K * z_diff;
        let p = p - K * S * K.transpose();

        // calculate nis
        let nis = match m.clone() {
            Some(m) => m.nis(&z_pred, &S),
            None => 0.0,
        };

        (x, p, nis)
    }

    fn radar_update(
        &self,
        m: Option<RadarMeasurement>,
        x_sig_pred: SigmaPoints,
        x: StateVector,
        p: CovarMatrix,
    ) -> (StateVector, CovarMatrix, f64) {
        let z = match m.clone() {
            Some(m) => RadarStateVector::new(m.rho, m.theta, m.rho_dot),
            None => RadarStateVector::zeros(),
        };

        debug!("radar z: {:?}", z);

        //
        // predict measurement
        //

        // sigma points radar measurement space
        let mut Z_sig = RadarSigmaPoints::zeros();

        for i in 0..Z_sig.cols() {
            let p_x = x_sig_pred[(0, i)];
            let p_y = x_sig_pred[(1, i)];
            let v = x_sig_pred[(2, i)];
            let yaw = x_sig_pred[(3, i)];
            let v1 = v * yaw.cos();
            let v2 = v * yaw.sin();

            // measurement model
            Z_sig[(0, i)] = (p_x * p_x + p_y * p_y).sqrt();
            Z_sig[(1, i)] = p_y.atan2(p_x);
            if Z_sig[(0, i)] != 0.0 {
                Z_sig[(2, i)] = (p_x * v1 + p_y * v2) / Z_sig.column(i)[0]; // r_dot
            } else {
                Z_sig[(2, i)] = 0.0;
            }
        }

        // predict measurement and covariance
        let mut S = RadarCovarMatrix::zeros();

        // measurement covariance matrix S
        for i in 1..Z_sig.cols() {
            let z_diff = Z_sig.column(i) - Z_sig.column(0);
            S += self.weights[i] * z_diff * z_diff.transpose();
        }

        // add measurement noise
        match &self.radar_sensor {
            Some(sensor) => S += sensor.noise_covar_matrix(),
            None => warn!("no lidar sensor defined - unable to add noise to covariance matrix"),
        };

        // mean predicted measurement
        let mut z_pred = RadarStateVector::zeros();
        for i in 0..Z_sig.cols() {
            z_pred += self.weights[i] * Z_sig.column(i);
        }

        //
        // Update state
        //
        let mut Tc = RadarCrossCorrelationMatrix::zeros();
        let n_sig_cols = Z_sig.cols();

        for i in 1..n_sig_cols {
            let mut z_diff = Z_sig.column(i) - Z_sig.column(0);
            z_diff[1] = negative_normalize(z_diff[1]);
            let mut x_diff =
                x_sig_pred.fixed_slice::<N_X, 1>(0, i) - x_sig_pred.fixed_slice::<N_X, 1>(0, 0);
            x_diff[3] = negative_normalize(x_diff[3]);
            Tc += self.weights[i] * x_diff * z_diff.transpose();
        }

        // Kalma gain K
        let K: RadarKalmanGain = Tc
            * match S.try_inverse() {
                Some(s_inverse) => s_inverse,
                None => S,
            };

        // residual
        let z_diff = z - z_pred;

        // update state mean and covariance matrix
        let x = x + K * z_diff;
        let p = p - K * S * K.transpose();

        // calculate nis
        let nis = match m {
            Some(m) => m.nis(&z_pred, &S),
            None => 0.0,
        };

        (x, p, nis)
    }
}
