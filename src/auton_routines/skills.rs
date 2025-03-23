use alloc::{boxed::Box, rc::Rc};
use core::time::Duration;

use async_trait::async_trait;
use nalgebra::Vector2;
use vexide::io::println;

use super::AutonRoutine;
use crate::{
    auton_routines::run_macros::alliance_stake::run_alliance_stake,
    differential::{
        motions::{
            angular::{TurnToParameters, TurnToTarget},
            linear::{MoveRelativeParameters, MoveToPointParameters},
            ramsete::{RAMSETEHybridParameters, RAMSETETarget},
        },
        pose::Pose,
    },
    subsystems::ladybrown::LadyBrownState,
    utils::{math::AngleExt, AllianceColor, FIELD_WALL},
    Robot,
};
pub struct Skills;
#[async_trait(?Send)]
impl AutonRoutine for Skills {
    fn name() -> &'static str {
        "Skills"
    }

    fn color() -> crate::utils::AllianceColor {
        crate::utils::AllianceColor::Red
    }

    fn symbol() -> &'static str {
        "skl "
    }

    async fn run(&self, robot: &mut Robot) {
        let chassis = robot.chassis.clone();
        let intake = robot.intake.clone();
        let ladybrown_arm = robot.ladybrown_arm.clone();
        chassis.set_filter_state(true).await;
        Rc::clone(&chassis)
            .set_pose(Pose::new(-62.0, 0.0, -90.0.hdg_deg()))
            .await;

        intake.lock().await.spin();
        vexide::time::sleep(Duration::from_millis(500)).await;
        intake.lock().await.stop();

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-52.0, -10.0))
            .params(params_move_to_point!(forwards: false, min_linear_speed: 0.2, early_exit_range: 2.0,))
            .call()
            .await;

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::Point(Vector2::new(-49.0, -24.0)))
            .run_async(false)
            .call()
            .await;
        robot.clamp_main.set_state(true);

        intake.lock().await.spin();

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::Point(Vector2::new(-26.0, -23.0)))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .early_exit_range(2.0.deg())
                    .min_speed(0.2)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-28.0, -23.0))
            .params(
                MoveToPointParameters::builder()
                    .forwards(false)
                    .early_exit_range(2.0)
                    .min_linear_speed(0.1)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::Angle(135.0.hdg_deg()))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .early_exit_range(2.0.deg())
                    .min_speed(0.3)
                    .build(),
            )
            .call()
            .await;

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(23.0, -47.0))
            .params(params_move_to_point!(forwards: false, min_linear_speed: 0.2, early_exit_range: 2.0,))
            .call()
            .await;

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(46.0, -47.0))
            .params(params_move_to_point!(forwards: false, max_linear_speed: 0.7))
            .call()
            .await;
        chassis.wait_until(10.0).await;

        intake.lock().await.set_optical_callback(Box::new({
            let lady_brown = ladybrown_arm.clone();
            move |color: AllianceColor| {
                if color == Skills::color() {
                    lady_brown.borrow_mut().set_state(LadyBrownState::Load);
                    return true;
                }
                false
            }
        }));

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(0.0, -42.0))
            .params(params_move_to_point!(min_linear_speed: 0.2, early_exit_range: 1.0,))
            .call()
            .await;
        chassis.wait_until(5.0).await;
        ladybrown_arm.borrow_mut().set_state(LadyBrownState::Load);
        intake.lock().await.clear_optical_callback();
        chassis.wait_until_complete().await;
        intake.lock().await.set_velocity(-0.2);
        ladybrown_arm.borrow_mut().set_state(LadyBrownState::LoadUp);

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(0.0, -58.0))
            .timeout(Duration::from_millis(1000))
            .params(params_turn_to!(forwards: false, min_speed: 0.3, early_exit_range: 10.0.deg(),))
            .call()
            .await;
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target(RAMSETETarget::pose(0.0, -58.0, 180.0.hdg_deg()))
            .timeout(Duration::from_millis(1000))
            .params(params_ramsete_h!(forwards: false, max_linear_speed: 0.6))
            .call()
            .await;
        vexide::time::sleep(Duration::from_millis(100)).await;
        intake.lock().await.spin();
        chassis.wait_until_complete().await;
        ladybrown_arm
            .borrow_mut()
            .set_state(LadyBrownState::Neutral);
        vexide::time::sleep(Duration::from_millis(500)).await;
        Rc::clone(&chassis)
            .move_relative()
            .distance(8.0)
            .params(
                MoveRelativeParameters::builder()
                    .early_exit_range(2.0)
                    .min_linear_speed(0.4)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(-24, -46))
            .params(
                TurnToParameters::builder()
                    .early_exit_range(2.0.deg())
                    .min_speed(0.2)
                    .forwards(false)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-24.0, -46.0))
            .params(
                MoveToPointParameters::builder()
                    .early_exit_range(2.0)
                    .min_linear_speed(0.2)
                    .forwards(false)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-58.0, -46.0))
            .params(MoveToPointParameters::builder().forwards(false).build())
            .call()
            .await;

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(-48, -57))
            .params(
                TurnToParameters::builder()
                    .early_exit_range(2.0.deg())
                    .min_speed(0.2)
                    .forwards(false)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-48.0, -57.0))
            .params(
                MoveToPointParameters::builder()
                    .early_exit_range(2.0)
                    .min_linear_speed(0.2)
                    .forwards(false)
                    .build(),
            )
            .run_async(false)
            .call()
            .await;
        intake.lock().await.set_optical_callback(Box::new({
            let intake = intake.clone();
            move |color: AllianceColor| {
                if color == Skills::color() {
                    vexide::task::spawn({
                        let intake = intake.clone();

                        async move {
                            intake.lock().await.stop();
                        }
                    })
                    .detach();
                    return true;
                }
                false
            }
        }));
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-35.0, -58.0))
            .params(MoveToPointParameters::builder().forwards(false).build())
            .run_async(false)
            .call()
            .await;
        intake.lock().await.stop();
        intake.lock().await.clear_optical_callback();
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-FIELD_WALL as f64, -60.0))
            .call()
            .await;
        chassis.wait_until(5.0).await;
        robot.clamp_main.set_state(false);
        Rc::clone(&chassis)
            .move_relative()
            .distance(-10.0)
            .params(
                MoveRelativeParameters::builder()
                    .min_linear_speed(0.4)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-48.0, 23.0))
            .params(
                MoveToPointParameters::builder()
                    .min_linear_speed(0.2)
                    .early_exit_range(20.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-48.0, 23.0))
            .params(
                MoveToPointParameters::builder()
                    .max_linear_speed(0.4)
                    .build(),
            )
            .run_async(false)
            .call()
            .await;

        // Mogo 2. There's already a ring in the intake from the optical callback.
        robot.clamp_main.set_state(true);
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(-23.0, 23.0))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .min_speed(0.3)
                    .early_exit_range(5.0.deg())
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-23.0, 23.0))
            .params(
                MoveToPointParameters::builder()
                    .forwards(false)
                    .min_linear_speed(0.2)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        intake.lock().await.spin();
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(-23.0, 46.0))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .min_speed(0.3)
                    .early_exit_range(5.0.deg())
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-23.0, 46.0))
            .params(
                MoveToPointParameters::builder()
                    .forwards(false)
                    .min_linear_speed(0.2)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(-58.0, 46.0))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .min_speed(0.3)
                    .early_exit_range(5.0.deg())
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-58.0, 46.0))
            .params(
                MoveToPointParameters::builder()
                    .forwards(false)
                    .min_linear_speed(0.2)
                    .max_linear_speed(0.6)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;

        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(-48, 57))
            .params(
                TurnToParameters::builder()
                    .early_exit_range(2.0.deg())
                    .min_speed(0.2)
                    .forwards(false)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-48.0, 57.0))
            .params(
                MoveToPointParameters::builder()
                    .early_exit_range(2.0)
                    .min_linear_speed(0.2)
                    .forwards(false)
                    .build(),
            )
            .run_async(false)
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-35.0, 58.0))
            .params(MoveToPointParameters::builder().forwards(false).build())
            .run_async(false)
            .call()
            .await;

        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-FIELD_WALL as f64, 60.0))
            .call()
            .await;
        chassis.wait_until(5.0).await;
        intake.lock().await.stop();
        robot.clamp_main.set_state(false);
        Rc::clone(&chassis)
            .move_relative()
            .distance(-10.0)
            .params(
                MoveRelativeParameters::builder()
                    .min_linear_speed(0.4)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(-1.0, 43.0))
            .params(
                MoveToPointParameters::builder()
                    .forwards(false)
                    .min_linear_speed(0.1)
                    .early_exit_range(2.0)
                    .build(),
            )
            .call()
            .await;
        chassis.wait_until(30.0).await;
        ladybrown_arm.borrow_mut().set_state(LadyBrownState::Load);
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target(RAMSETETarget::pose(0.0, 58.0, 0.0.hdg_deg()))
            .timeout(Duration::from_millis(1000))
            .params(params_ramsete_h!(forwards: false, max_linear_speed: 0.6))
            .call()
            .await;
        intake.lock().await.spin();
        chassis.wait_until_complete().await;
        vexide::time::sleep(Duration::from_millis(600)).await;
        intake.lock().await.set_velocity(-0.2);
        ladybrown_arm
            .borrow_mut()
            .set_state(LadyBrownState::Neutral);
        vexide::time::sleep(Duration::from_millis(500)).await;
        Rc::clone(&chassis)
            .move_relative()
            .distance(8.0)
            .params(
                MoveRelativeParameters::builder()
                    .early_exit_range(2.0)
                    .min_linear_speed(0.4)
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(23.0, 23.0))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .min_speed(0.3)
                    .early_exit_range(7.0.deg())
                    .build(),
            )
            .call()
            .await;
        intake.lock().await.spin();
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(23.0, 23.0))
            .params(MoveToPointParameters::builder().forwards(false).build())
            .call()
            .await;
        chassis.wait_until(20.0).await;
        ladybrown_arm.borrow_mut().set_state(LadyBrownState::Load);
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(47.0, 0.0))
            .params(
                TurnToParameters::builder()
                    .min_speed(0.3)
                    .early_exit_range(5.0.deg())
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .move_to_point()
            .target(Vector2::new(47.0, 0.0))
            .params(
                MoveToPointParameters::builder()
                    .max_linear_speed(0.4)
                    .build(),
            )
            .run_async(false)
            .call()
            .await;
        chassis.wait_until_complete().await;
        robot.clamp_main.set_state(true);
        Rc::clone(&chassis)
            .turn_to()
            .target(TurnToTarget::point(62.0, 0.0))
            .params(
                TurnToParameters::builder()
                    .forwards(false)
                    .min_speed(0.3)
                    .early_exit_range(5.0.deg())
                    .build(),
            )
            .call()
            .await;
        Rc::clone(&chassis)
            .ramsete_hybrid()
            .target(RAMSETETarget::pose(62.0, 0.0, 90.0.hdg_deg()))
            .timeout(Duration::from_millis(1000))
            .params(RAMSETEHybridParameters::builder().forwards(false).build())
            .call()
            .await;
        chassis.wait_until_complete().await;
        run_alliance_stake(robot).await;
        chassis.wait_until_complete().await;
        intake.lock().await.spin();

        println!("{}", Skills::color().get_name());
    }
}
