use alloc::boxed::Box;

use async_trait::async_trait;
use lamlib_rs::utils::AllianceColor;

use crate::Robot;
#[macro_export]
macro_rules! create_route_from_routine {
    ($auton_type:path) => {{
        use $crate::auton_routines::AutonRoutine;
        autons::simple::Route::new(<$auton_type>::name(), |robot: &mut Robot| {
            let auton = Rc::new($auton_type);
            info!("Autonomous starting: {}", <$auton_type>::name());
            Box::pin(async move { auton.run(robot).await })
        })
    }};
}
pub use create_route_from_routine;

pub mod negative_elims;
pub mod partials;
pub mod skills;
pub mod test;

// Append routines here as needed.
#[macro_export]
macro_rules! create_routine_array {
    () => {
        [
            create_route_from_routine!($crate::auton_routines::negative_elims::RedNegativeElims),
            create_route_from_routine!($crate::auton_routines::negative_elims::BlueNegativeElims),
            create_route_from_routine!($crate::auton_routines::skills::Skills),
            create_route_from_routine!($crate::auton_routines::test::Test),
        ]
    };
}
pub use create_routine_array;

#[async_trait(?Send)]
pub trait AutonRoutine {
    fn name() -> &'static str;
    fn symbol() -> &'static str;
    fn color() -> AllianceColor;
    async fn run(&self, robot: &mut Robot);
}
