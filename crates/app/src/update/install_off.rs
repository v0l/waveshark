use super::{Asset, INSTALL, Install};

pub fn install(_: Asset) {
    *INSTALL.write() = Install::Failed("this build does not install updates".into());
}
