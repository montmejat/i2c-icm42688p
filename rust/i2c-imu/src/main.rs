use libgpiod::line;

mod icm42688p;

fn main() -> libgpiod::Result<()> {
    let mut icm = icm42688p::Icm42688p::new("/dev/i2c-1", 0x68);
    icm.change_accel_scale(icm42688p::AccelScale::Fs4g);
    icm.change_accel_odr(icm42688p::SamplingRate::Odr12_5);
    icm.change_gyro_scale(icm42688p::GyroScale::Fs2000dps);
    icm.change_gyro_odr(icm42688p::SamplingRate::Odr12_5);
    icm.enable_data_interuption();

    let chip_path = "/dev/gpiochip0";
    let line_offset = 4;

    let mut lsettings = line::Settings::new()?;
    lsettings.set_edge_detection(Some(line::Edge::Rising))?;

    let mut lconfig = line::Config::new()?;
    lconfig.add_line_settings(&[line_offset], lsettings)?;

    let mut rconfig = libgpiod::request::Config::new()?;
    rconfig.set_consumer("imu-data-event")?;

    let chip = libgpiod::chip::Chip::open(&chip_path)?;
    let request = chip.request_lines(Some(&rconfig), &lconfig)?;

    let mut buffer = libgpiod::request::Buffer::new(1)?;
    loop {
        let events = request.read_edge_events(&mut buffer)?;

        for event in events {
            let event = event?;
            icm.read_data();

            println!(
                "timestamp: {}, temp: {}, accel: {:?}, gyro: {:?}",
                event.timestamp().as_nanos(),
                icm.temperature,
                icm.accel,
                icm.gyro,
            );
        }
    }
}
