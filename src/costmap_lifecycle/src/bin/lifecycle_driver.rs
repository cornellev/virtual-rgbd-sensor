use anyhow::{anyhow, Result};
use rclrs::*;
use lifecycle_msgs::srv::ChangeState;
use lifecycle_msgs::msg::Transition;
use sensor_msgs::msg::PointCloud2;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

type ChangeStateRequest = <ChangeState as ServiceIDL>::Request;
type ChangeStateResponse = <ChangeState as ServiceIDL>::Response;

const TRANSITION_CONFIGURE: u8 = 1;
const TRANSITION_ACTIVATE: u8 = 3;

async fn call_transition(client: &Client<ChangeState>, id: u8, label: &str) -> Result<()> {
    let request = ChangeStateRequest {
        transition: Transition { id, label: label.to_string() },
    };
    println!("Requesting transition: {label}");
    let response: ChangeStateResponse = client.call(&request)?.await?;
    if !response.success {
        return Err(anyhow!("transition '{label}' reported failure"));
    } else {
        println!("Transition '{label}' succeeded.");
    }
    Ok(())
}

fn main() -> Result<()> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("lifecycle_driver")?;

    let client = node.create_client::<ChangeState>("/costmap/costmap/change_state")?;

    println!("Waiting for /costmap/costmap/change_state service...");
    for _ in 0..150 {
        if client.service_is_ready()? {
            println!("Service ready.");
            let promise = executor.commands().run(async move {
                if let Err(error) = async {
                    call_transition(&client, TRANSITION_CONFIGURE, "configure").await?;
                    std::thread::sleep(Duration::from_millis(500));
                    call_transition(&client, TRANSITION_ACTIVATE, "activate").await
                }
                .await
                {
                    eprintln!("Lifecycle transition failed: {error:#}");
                    std::process::exit(1);
                }
            });
            executor
                .spin(SpinOptions::new().until_promise_resolved(promise))
                .first_error()?;
            println!("Costmap is now active.");

            let received = Arc::new(AtomicU64::new(0));
            let received_callback = Arc::clone(&received);
            let _pointcloud_subscription = node.create_subscription::<PointCloud2, _>(
                "/rslidar_points",
                move |message: PointCloud2| {
                    let count = received_callback.fetch_add(1, Ordering::Relaxed) + 1;
                    println!(
                        "PointCloud2 received #{count}: stamp={}.{:09}, frame='{}', \
                         points={} ({}x{}), point_step={}, bytes={}",
                        message.header.stamp.sec,
                        message.header.stamp.nanosec,
                        message.header.frame_id,
                        message.width as u64 * message.height as u64,
                        message.width,
                        message.height,
                        message.point_step,
                        message.data.len(),
                    );
                },
            )?;

            println!("Monitoring /rslidar_points; one line will be printed for every PointCloud2.");
            executor.spin(SpinOptions::default()).first_error()?;
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    Err(anyhow!(
        "timed out waiting for /costmap/costmap/change_state"
    ))
}