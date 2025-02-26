use alloc::{rc::Rc, vec, vec::Vec};
use core::time::Duration;

use nalgebra::Vector3;
use vexide::{
    core::sync::Mutex,
    prelude::{Float, InertialSensor, Task},
};

use super::odom_wheels::OdomWheel;
use crate::tracking::abstract_tracking::Tracking;
pub struct OdomInertial {
    /// A mutex containing the inertial sensor.
    pub inertial: Rc<Mutex<InertialSensor>>,

    /// For imu scaling.
    /// Use larger values to make the imu return a higher value.
    pub scalar: f64,

    /// How much to prioritize this sensor's output.
    /// Values of all inertial weights are normalized to 1.
    pub weight: f64,
}
impl OdomInertial {
    pub fn new(inertial: Rc<Mutex<InertialSensor>>, scalar: f64, weight: f64) -> Self {
        Self {
            inertial,
            scalar,
            weight,
        }
    }
}
pub struct OdomSensors {
    imus: Vec<Rc<OdomInertial>>,
    horizontals: Vec<Rc<OdomWheel>>,
    verticals: Vec<Rc<OdomWheel>>,

    /// How much imu weighted rotation affects total rotation.
    /// Values of all weights are normalized to 1.
    imu_angle_weight: f64,

    /// How much pairs of horizontals weighted rotation affects total rotation.
    /// Values of all weights are normalized to 1.
    /// If you only have one horizontal, this does nothing.
    horizontal_angle_weight: f64,

    /// How much pairs of verticals weighted rotation affects total rotation.
    /// Values of all weights are normalized to 1.
    /// If you only have one vertical, this does nothing.
    vertical_angle_weight: f64,
}
impl OdomSensors {
    pub fn new(
        imus: Vec<Rc<OdomInertial>>,
        horizontals: Vec<Rc<OdomWheel>>,
        verticals: Vec<Rc<OdomWheel>>,
        imu_weight: f64,
        horizontal_weight: f64,
        vertical_weight: f64,
    ) -> Self {
        Self {
            imus,
            horizontals,
            verticals,
            imu_angle_weight: imu_weight,
            horizontal_angle_weight: horizontal_weight,
            vertical_angle_weight: vertical_weight,
        }
    }
    pub fn imu_count(&self) -> usize {
        self.imus.len()
    }
    pub fn horizontals_count(&self) -> usize {
        self.horizontals.len()
    }
    pub fn verticals_count(&self) -> usize {
        self.verticals.len()
    }
}
pub struct OdomTracking {
    task: Option<Task<()>>,
    sensors: Rc<OdomSensors>,
    tracked_pose: Vector3<f64>,
    delta_global_pose: Vector3<f64>,
    prev_imu_angles: Vec<Option<f64>>,
    prev_horizontal_distances: Vec<Option<f64>>,
    prev_vertical_distances: Vec<Option<f64>>,
}

macro_rules! avg_valid {
    ($vec:expr) => {{
        let (sum, count) = $vec
            .iter()
            .filter_map(|&x| x)
            .fold((0.0, 0), |(sum, count), x| (sum + x, count + 1));

        if count > 0 {
            sum / count as f64
        } else {
            0.0
        }
    }};
}

impl OdomTracking {
    pub fn new(sensors: Rc<OdomSensors>) -> Self {
        Self {
            task: None,
            sensors: sensors.clone(),
            tracked_pose: Vector3::new(0.0, 0.0, 0.0),
            delta_global_pose: Vector3::new(0.0, 0.0, 0.0),
            prev_imu_angles: vec![None; sensors.imu_count()],
            prev_horizontal_distances: vec![None; sensors.horizontals_count()],
            prev_vertical_distances: vec![None; sensors.verticals_count()],
        }
    }
    async fn update(&mut self) {
        // Use a basic fusion of inertials and tracking wheels to get delta theta.
        let mut imu_angles: Vec<Option<f64>> = Vec::new();
        let mut delta_imu_angles: Vec<Option<f64>> = Vec::new();
        let mut delta_imu_weights: Vec<Option<f64>> = Vec::new();
        for (i, imu) in self.sensors.imus.iter().enumerate() {
            let imu_lock = imu.inertial.lock().await;
            let rotation = match (imu_lock.rotation(), &self.prev_imu_angles[i]) {
                (Ok(rotation), _) => Some(-rotation * imu.scalar),
                (Err(_), Some(rotation)) => Some(*rotation),
                (Err(_), None) => None,
            };
            imu_angles.push(rotation);
        }
        for (new_angle, (prev_angle, imu)) in imu_angles
            .iter()
            .zip(self.prev_imu_angles.iter().zip(self.sensors.imus.iter()))
        {
            match (new_angle, prev_angle) {
                (Some(new_angle), Some(prev_angle)) => {
                    delta_imu_angles.push(Some(new_angle - prev_angle));
                    delta_imu_weights.push(Some(imu.weight));
                }
                _ => {
                    delta_imu_angles.push(None);
                    delta_imu_weights.push(None);
                }
            }
        }
        let delta_imu_total_weight: f64 = delta_imu_weights.iter().filter_map(|w| *w).sum();
        let delta_imu_angle: Option<f64> = if delta_imu_total_weight > 0.0 {
            Some(
                delta_imu_angles
                    .iter()
                    .zip(&delta_imu_weights)
                    .filter_map(|(delta, weight)| match (delta, weight) {
                        (Some(d), Some(w)) => Some(d * w),
                        _ => None,
                    })
                    .sum::<f64>()
                    / delta_imu_total_weight,
            )
        } else {
            None
        };

        self.prev_imu_angles = imu_angles;

        let mut horizontal_distances: Vec<Option<f64>> = Vec::new();
        for (i, tracking_wheel) in self.sensors.horizontals.iter().enumerate() {
            let distance_traveled = match (
                tracking_wheel.distance_traveled(),
                &self.prev_horizontal_distances[i],
            ) {
                (Some(distance), _) => Some(distance),
                (None, Some(distance)) => Some(*distance),
                (None, None) => None,
            };
            horizontal_distances.push(distance_traveled);
        }
        let delta_horizontal_distances: Vec<Option<f64>> = horizontal_distances
            .iter()
            .zip(&self.prev_horizontal_distances)
            .map(
                |(new_distance, prev_distance)| match (new_distance, prev_distance) {
                    (Some(new), Some(old)) => Some(new - old),
                    _ => None,
                },
            )
            .collect();

        let mut delta_horizontal_angles: Vec<f64> = Vec::new();

        // If there is less than 2 horizontal wheels, the range is empty.
        for i in 0..delta_horizontal_distances.len() {
            for j in i + 1..delta_horizontal_distances.len() {
                if let (Some(delta_distance_1), Some(delta_distance_2)) =
                    (delta_horizontal_distances[i], delta_horizontal_distances[j])
                {
                    let offset_1 = self.sensors.horizontals[i].offset();
                    let offset_2 = self.sensors.horizontals[j].offset();
                    delta_horizontal_angles
                        .push((delta_distance_1 - delta_distance_2) / (offset_1 - offset_2));
                }
            }
        }
        let delta_horizontal_angle: Option<f64> = if delta_horizontal_angles.is_empty() {
            None
        } else {
            Some(delta_horizontal_angles.iter().sum::<f64>() / delta_horizontal_angles.len() as f64)
        };

        let mut vertical_distances: Vec<Option<f64>> = Vec::new();
        for (i, tracking_wheel) in self.sensors.verticals.iter().enumerate() {
            let distance_traveled = match (
                tracking_wheel.distance_traveled(),
                &self.prev_vertical_distances[i],
            ) {
                (Some(distance), _) => Some(distance),
                (None, Some(distance)) => Some(*distance),
                (None, None) => None,
            };
            vertical_distances.push(distance_traveled);
        }
        let delta_vertical_distances: Vec<Option<f64>> = vertical_distances
            .iter()
            .zip(self.prev_vertical_distances.iter())
            .map(
                |(new_distance, prev_distance)| match (new_distance, prev_distance) {
                    (Some(new), Some(old)) => Some(new - old),
                    _ => None,
                },
            )
            .collect();

        let mut delta_vertical_angles: Vec<f64> = Vec::new();

        // If there is less than 2 vertical wheels, the range is empty.
        for i in 0..delta_vertical_distances.len() {
            for j in i + 1..delta_vertical_distances.len() {
                if let (Some(delta_distance_1), Some(delta_distance_2)) =
                    (delta_vertical_distances[i], delta_vertical_distances[j])
                {
                    let offset_1 = self.sensors.verticals[i].offset();
                    let offset_2 = self.sensors.verticals[j].offset();
                    delta_vertical_angles
                        .push((delta_distance_1 - delta_distance_2) / (offset_1 - offset_2));
                }
            }
        }
        let delta_vertical_angle: Option<f64> = if delta_vertical_angles.is_empty() {
            None
        } else {
            Some(delta_vertical_angles.iter().sum::<f64>() / delta_vertical_angles.len() as f64)
        };
        let total_weight: f64 = if delta_imu_angle.is_some() {
            self.sensors.imu_angle_weight
        } else {
            0.0
        } + if delta_horizontal_angle.is_some() {
            self.sensors.horizontal_angle_weight
        } else {
            0.0
        } + if delta_vertical_angle.is_some() {
            self.sensors.vertical_angle_weight
        } else {
            0.0
        };
        let delta_angle = if let Some(delta_imu_angle) = delta_imu_angle {
            delta_imu_angle * self.sensors.imu_angle_weight / total_weight
        } else {
            0.0
        } + if let Some(delta_horizontal_angle) = delta_horizontal_angle {
            delta_horizontal_angle * self.sensors.horizontal_angle_weight / total_weight
        } else {
            0.0
        } + if let Some(delta_vertical_angle) = delta_vertical_angle {
            delta_vertical_angle * self.sensors.vertical_angle_weight / total_weight
        } else {
            0.0
        };
        let new_angle: f64 = self.tracked_pose.z + delta_angle;
        // Assume this is the average for constant velocity.
        // It should be close enough (adequate) if update is fast enough.
        let avg_angle: f64 = self.tracked_pose.z + delta_angle / 2.0;
        self.prev_horizontal_distances = horizontal_distances;
        self.prev_vertical_distances = vertical_distances;

        let local_delta_horizontal_distance: f64;
        let local_delta_vertical_distance: f64;

        if delta_angle == 0.0 {
            local_delta_horizontal_distance = avg_valid!(delta_horizontal_distances);
            local_delta_vertical_distance = avg_valid!(delta_vertical_distances);
        } else {
            local_delta_horizontal_distance = avg_valid!(delta_horizontal_distances
                .iter()
                .zip(self.sensors.horizontals.iter())
                .map(|(&d, sensor)| d
                    .map(|d| 2.0 * (delta_angle / 2.0).sin() * (d / delta_angle - sensor.offset())))
                .collect::<Vec<Option<f64>>>());

            local_delta_vertical_distance = avg_valid!(delta_vertical_distances
                .iter()
                .zip(self.sensors.verticals.iter())
                .map(|(&d, sensor)| d
                    .map(|d| 2.0 * (delta_angle / 2.0).sin() * (d / delta_angle - sensor.offset())))
                .collect::<Vec<Option<f64>>>());
        }
        let cos_value: f64 = avg_angle.cos();
        let sin_value: f64 = avg_angle.sin();

        // At 0 degrees (facing right): (vertical, -horizontal)
        // At 90 degrees (facing up): (horizontal, vertical)
        self.delta_global_pose = nalgebra::Vector3::new(
            local_delta_vertical_distance * cos_value + local_delta_horizontal_distance * sin_value,
            local_delta_vertical_distance * sin_value - local_delta_horizontal_distance * cos_value,
            delta_angle,
        );

        self.tracked_pose.x += self.delta_global_pose.x;
        self.tracked_pose.y += self.delta_global_pose.y;
        self.tracked_pose.z = new_angle;
    }
}

impl Tracking for OdomTracking {
    fn position(&mut self) -> Vector3<f64> {
        self.tracked_pose
    }
    fn set_position(&mut self, position: &Vector3<f64>) {
        self.tracked_pose = *position;
    }
    async fn init(&mut self, self_rc: Rc<Mutex<Self>>) {
        self.sensors
            .horizontals
            .iter()
            .for_each(|tracking_wheel| tracking_wheel.init());
        self.sensors
            .verticals
            .iter()
            .for_each(|tracking_wheel| tracking_wheel.init());
        for imu in &self.sensors.imus {
            let imu_lock_value = &mut imu.inertial.lock().await;
            let calibration_result_1 = imu_lock_value.calibrate().await;
            match &calibration_result_1 {
                Ok(_) => {}
                Err(_) => {
                    let _ = imu_lock_value.calibrate().await;
                    let _ = imu_lock_value.set_rotation(0.0);
                }
            }
        }
        self.task = Some(vexide::async_runtime::spawn({
            let self_rc = self_rc.clone();
            async move {
                loop {
                    {
                        vexide::async_runtime::time::sleep(Duration::from_millis(10)).await;
                        let mut self_lock = self_rc.lock().await;
                        self_lock.update().await;
                    }
                }
            }
        }));
    }
}
