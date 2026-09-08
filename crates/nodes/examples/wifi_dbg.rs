fn main() {
    println!("width {}", dsp::wifi::ofdm::CHANNEL_WIDTH_HZ);
    let ch = nodes::wifi_nodes::starting_channels();
    println!("starting {:?}", ch.iter().map(|c| c / 1e6).collect::<Vec<_>>());
    for center in [2427e6, 2437e6, 2462e6] {
        let s = dsp::wifi::WifiSpan::new(20e6, center, &ch, Default::default());
        println!(
            "{} MHz -> {:?}",
            center / 1e6,
            s.map(|s| s.channels().iter().map(|c| c / 1e6).collect::<Vec<_>>())
        );
    }
}
