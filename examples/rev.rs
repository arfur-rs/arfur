//! Example usage of a REV SparkMax wired at CAN ID 16.

use arfur_rev::prelude::*;
use arfur_wpilib::prelude::*;

fn main() -> Result<()> {
    let robot = RobotBuilder::default().initialize()?;
    let mut spark = SparkMax::new(robot, 16);

    spark.reset();

    loop {
        spark.set_percentage(0.5);
        std::thread::sleep(std::time::Duration::from_secs(1));
        spark.set_percentage(0.);
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
