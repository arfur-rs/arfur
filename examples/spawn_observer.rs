use arfur::prelude::*;

fn main() -> Result<()> {
    let _robot = RobotBuilder::default().initialize();

    // The above works great, but alone, the DS will disable immediately. The
    // DS wants constant approval that the program is working. This is why we
    // spawn a special utility task in the background, that runs the
    // corresponding observe functions every 50 ms.
    std::thread::spawn(arfur::wpilib::util::create_observer());

    loop {}
}
