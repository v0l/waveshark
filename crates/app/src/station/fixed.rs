pub fn set_source(_: Option<gps::Transport>) {}

pub fn fix() -> Option<gps::Fix> {
    None
}

pub fn connected() -> bool {
    false
}

pub fn fixes() -> u64 {
    0
}

pub fn sky() -> Option<gps::Sky> {
    None
}
